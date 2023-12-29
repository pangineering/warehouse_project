import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped, Twist
import math
import time
import tf2_ros
from std_msgs.msg import Empty
from action_msgs.msg import GoalStatus

class MoveShelfToShip(Node):
    def __init__(self):
        super().__init__('move_shelf_to_ship')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Action server not available, waiting again...')

        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot/cmd_vel',
            10  # Adjust queue size as needed
        )
        self.elevator_up_publisher = self.create_publisher(
            Empty,
            '/elevator_up',
            10  # Adjust queue size as needed
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.loaded = False

    def move_back(self):
        # Create a Twist message for controlling robot movement
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -0.2  # Adjust linear velocity as needed
        cmd_vel_msg.angular.z = 0.0  # No rotation

        # Publish the Twist message to control the robot's movement
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Wait for the robot to reach the target position (using time.sleep)
        time.sleep(8)  # Example sleep duration (1 second)

        # Stop the robot by publishing zero velocity command
        cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def load(self):
        self.send_goal(5.68461, -0.743197, 0.0, 0.0, 0.0, -0.693068, 0.720872)
        time.sleep(30)

    def get_shelf(self):
        self.load_shelf()
        time.sleep(10)
        self.move_back()
        time.sleep(5)
        
    def go_to_shipping(self):
        self.send_goal(1.27959,-2.40654,0.0,0.0,0.0,0.999864,0.0164729)
    
    def execute(self):
        self.load()
        time.sleep(30)  # Assuming this is the loading time
        self.move_robot()  # Assuming this is the initial movement after loading
        time.sleep(2)  # Assuming the movement takes 2 seconds

        self.load_shelf()
        time.sleep(10)  # Assuming the shelf loading process takes 10 seconds

        self.move_back()
        time.sleep(20)  # Assuming the robot takes 4 seconds to move back

        self.go_to_shipping()

    def load_shelf(self):
        # Trigger the elevator up action by publishing a message to /elevator_up
        elevator_up_msg = Empty()
        self.elevator_up_publisher.publish(elevator_up_msg)
        self.get_logger().info("Triggered elevator up action.")

    def send_goal(self,x,y,z,qx,qy,qz,qw):
        goal_pose = NavigateToPose.Goal()
        goal_pose.pose.header.frame_id = 'map'
        
        # Define the desired position
        goal_pose.pose.pose.position.x = x
        goal_pose.pose.pose.position.y = y
        goal_pose.pose.pose.position.z = z
        
        # Define the desired orientation
        goal_pose.pose.pose.orientation = Quaternion(
            x=qx,
            y=qy,
            z=qz,
            w=qw
        )

        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}' + str(result))
        self.executor.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK:' + str(feedback) )

    def move_robot(self):
        self.get_logger().info('Moving forward...')
        # Create a Twist message for controlling robot movement
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.5  # Adjust linear velocity as needed
        cmd_vel_msg.angular.z = 0.0  # No rotation

        # Publish the Twist message to control the robot's movement
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Wait for the robot to reach the target position (using time.sleep)
        time.sleep(2.8)  # Example sleep duration (1 second)

        # Stop the robot by publishing zero velocity command
        cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)

def main(args=None):
    rclpy.init(args=args)
    move_shelf_to_ship = MoveShelfToShip()
    move_shelf_to_ship.execute()
    rclpy.spin(move_shelf_to_ship)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
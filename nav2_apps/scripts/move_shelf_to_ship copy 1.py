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


class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('Nav_To_Pose_Action_Client')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.executor = None
        self.elevator_up_publisher = self.create_publisher(
            Empty,
            '/elevator_up',
            10  # Adjust queue size as needed
        )
        self.elevator_down_publisher = self.create_publisher(
            Empty,
            '/elevator_down',
            10  # Adjust queue size as needed
        )
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/robot/cmd_vel',
            10  # Adjust queue size as needed
        )
        self.loaded = False
        self.laser_detected_legs = False
        self.init_robot_x = 0.0  # Initialize robot's x position
        self.init_robot_y = 0.0  # Initialize robot's y position
        self.intensity_threshold = 0.5
        self.leg1_x = 0.0
        self.leg1_y = 0.0
        self.leg2_x = 0.0
        self.leg2_y = 0.0
        self.latest_scan = None
        self.execute()
    
        #if self.loaded:
        #    self.load()

    def execute(self):
        self.send_goal(5.68461, -0.743197, 0.0, 0.0, 0.0, -0.693068, 0.720872)
        time.sleep(5)
        self.move_back()
        
        #self.wait_for_seconds(2)
        #self.loaded = True

        #self.send_goal(1.27959,-2.40654,0.0,0.0,0.0,0.999864,0.0164729)
        #time.sleep(10)
        #self.unload_shelf()
        #time.sleep(2)
        #self.move_back()
        #time.sleep(2)
        #self.send_goal(0.003,0.007,0.0,0.0,0.0,0.0,0.0)

    def load(self):
        self.load_shelf()
        time.sleep(2)
        self.move_back()
        time.sleep(3)

    def perform_final_approach_logic(self):
        if self.laser_detected_legs:

            center_x = (self.leg1_x + self.leg2_x) / 2.0
            center_y = (self.leg1_y + self.leg2_y) / 2.0

            cart_transform = TransformStamped()
            cart_transform.header.frame_id = "robot_front_laser_base_link"
            cart_transform.child_frame_id = "cart_frame"
            cart_transform.transform.translation.x = center_x
            cart_transform.transform.translation.y = center_y
            cart_transform.transform.translation.z = 0.0
            cart_transform.transform.rotation.x = 0.0
            cart_transform.transform.rotation.y = 0.0
            cart_transform.transform.rotation.z = 0.0
            cart_transform.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(cart_transform)
            self.get_logger().info("Published cart_frame transform")

            rclpy.spin_once(self, timeout_sec=0.1)
            tar_x = cart_transform.transform.translation.x
            self.get_logger().info(str(tar_x))



    def move_towards_transform(self, target_x, target_y):
        # Calculate the distance between the current robot position and the target position
        distance_to_target = math.sqrt((target_x - self.robot_x)**2 + (target_y - self.robot_y)**2)

        # Calculate the angle to the target position
        angle_to_target = math.atan2(target_y - self.robot_y, target_x - self.robot_x)

        # Implement your robot's control mechanisms to adjust its orientation and move towards the target position
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.5  # Adjust linear velocity as needed
        cmd_vel_msg.angular.z = 0.0  # No rotation

        # Publish the Twist message to control the robot's movement
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Wait for the robot to reach the target position (you may need to implement feedback control)
        time.sleep(0.5)

        # Stop the robot
        cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Update the robot's position
        self.robot_x = target_x
        self.robot_y = target_y

    def get_current_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            pose = PoseWithCovarianceStamped()
            pose.pose.pose.position = transform.transform.translation
            pose.pose.pose.orientation = transform.transform.rotation
            return pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.get_logger().error("Failed to get current robot pose.")
            return None

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

    def load_shelf(self):
        # Trigger the elevator up action by publishing a message to /elevator_up
        elevator_up_msg = Empty()
        self.elevator_up_publisher.publish(elevator_up_msg)
        self.get_logger().info("Triggered elevator up action.")

    def unload_shelf(self):
        # Trigger the elevator up action by publishing a message to /elevator_up
        elevator_down_msg = Empty()
        self.elevator_down_publisher.publish(elevator_down_msg)
        self.get_logger().info("Triggered elevator up action.")

    def move_robot(self):

        # Create a Twist message for controlling robot movement
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.5  # Adjust linear velocity as needed
        cmd_vel_msg.angular.z = 0.0  # No rotation

        # Publish the Twist message to control the robot's movement
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Wait for the robot to reach the target position (using time.sleep)
        time.sleep(5)  # Example sleep duration (1 second)

        # Stop the robot by publishing zero velocity command
        cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)

    def move_back(self):

        # Create a Twist message for controlling robot movement
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = -0.2  # Adjust linear velocity as needed
        cmd_vel_msg.angular.z = 0.0  # No rotation

        # Publish the Twist message to control the robot's movement
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Wait for the robot to reach the target position (using time.sleep)
        time.sleep(5)  # Example sleep duration (1 second)

        # Stop the robot by publishing zero velocity command
        cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)
    def detect_shelf_legs(self):
        # Implement your shelf leg detection logic using laser intensities here
        # You may need to process the scan data to identify shelf legs.
        # Return True if both legs are detected, otherwise False.

        # Placeholder logic for demonstration purposes (replace with actual detection)
        min_intensity = float('inf')
        if self.latest_scan:
            for i in range(len(self.latest_scan.intensities)):
                if self.latest_scan.intensities[i] < min_intensity:
                    min_intensity = self.latest_scan.intensities[i]
                    if min_intensity < self.intensity_threshold:
                        # Update the positions of detected legs based on their angles
                        # Replace these calculations with your actual logic
                        self.leg1_x = self.latest_scan.ranges[i] * \
                                      math.cos(self.latest_scan.angle_min + i * self.latest_scan.angle_increment)
                        self.leg1_y = self.latest_scan.ranges[i] * \
                                      math.sin(self.latest_scan.angle_min + i * self.latest_scan.angle_increment)
                        # Find the second leg
                        # You may need additional logic to confirm the second leg detection
                        # Replace these calculations with your actual logic
                        self.leg2_x = self.latest_scan.ranges[i] * \
                                      math.cos(self.latest_scan.angle_min + i * self.latest_scan.angle_increment)
                        self.leg2_y = self.latest_scan.ranges[i] * \
                                      math.sin(self.latest_scan.angle_min + i * self.latest_scan.angle_increment)
                        return True  # Both legs detected

        return False  # Legs not detected


    def laser_callback(self, msg):
        # Store the received laser scan message
        self.latest_scan = msg

        self.laser_detected_legs = self.detect_shelf_legs()
        self.get_logger().info(str(self.laser_detected_legs))


def main(args=None):
    rclpy.init(args=args)

    action_client = NavToPoseActionClient()

    #action_client.execute()

    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

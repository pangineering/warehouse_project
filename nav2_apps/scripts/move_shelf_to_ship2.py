# Assuming this class is saved in a file named 'navigator.py'

from geometry_msgs.msg import PoseStamped, Twist, PolygonStamped, Point32
from std_msgs.msg import Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration
import rclpy
import tf2_ros
from tf2_ros import TransformListener, TransformBroadcaster
import time  # Import the time module

class NavigatorDemo:
    def __init__(self):
        rclpy.init()
        self.navigator = BasicNavigator()

        self.cmd_vel_publisher = self.navigator.create_publisher(
            Twist,
            '/robot/cmd_vel',
            10  # Adjust queue size as needed
        )
        self.elevator_up_publisher = self.navigator.create_publisher(
            Empty,
            '/elevator_up',
            10  # Adjust queue size as needed
        )
        
        self.is_carrying_shelf = False  # Flag to indicate if the robot is carrying a shelf

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.navigator)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self.navigator)
        self.loaded = False
    
        self.publisher = self.navigator.create_publisher(PolygonStamped, '/local_costmap/published_footprint', 10)

    def setup_navigation(self):
        self.navigator.waitUntilNav2Active()

    def go_to_pose(self, goal_pose):
        self.navigator.goToPose(goal_pose)

    def is_task_complete(self):
        return self.navigator.isTaskComplete()

    def get_feedback(self):
        return self.navigator.getFeedback()

    def cancel_task(self):
        self.navigator.cancelTask()

    def get_result(self):
        return self.navigator.getResult()

    def lifecycle_shutdown(self):
        self.navigator.lifecycleShutdown()

    def run_demo(self, goal_pose):
        self.setup_navigation()
        self.go_to_pose(goal_pose)

        i = 0
        while not self.is_task_complete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i += 1
            feedback = self.get_feedback()
            if feedback and i % 5 == 0:
                print(
                    'Estimated time of arrival: '
                    + '{0:.0f}'.format(
                        Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                        / 1e9
                    )
                    + ' seconds.'
                )

                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    self.cancel_task()

        # Do something depending on the return code
        result = self.get_result()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        self.lifecycle_shutdown()

    def update_footprint_size(self):
        # Create a PolygonStamped message
        msg = PolygonStamped()
        msg.header.frame_id = 'map'  # Set the frame ID as needed

        # Define new points for the updated footprint size
        points = [
            Point32(x=5.0, y=-2.0, z=0.0),
            Point32(x=5.0, y=2.0, z=0.0),
            Point32(x=-5.0, y=2.0, z=0.0),
            Point32(x=-5.0, y=-2.0, z=0.0),
        ]

        msg.polygon.points = points

        # Publish the updated footprint size
        self.publisher.publish(msg)

    def load_shelf(self):
        # Trigger the elevator up action by publishing a message to /elevator_up
        elevator_up_msg = Empty()
        self.elevator_up_publisher.publish(elevator_up_msg)
        self.is_carrying_shelf = True

    def unload_shelf(self):
        # Add code to unload the shelf if needed
        self.is_carrying_shelf = False

    def move_robot(self):
        # Create a Twist message for controlling robot movement
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = 0.5  # Adjust linear velocity as needed
        cmd_vel_msg.angular.z = 0.0  # No rotation

        # Publish the Twist message to control the robot's movement
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # Wait for the robot to reach the target position (using time.sleep)
        time.sleep(2.8)  # Example sleep duration (2.8 seconds)

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
        time.sleep(8)  # Example sleep duration (8 seconds)

        # Stop the robot by publishing zero velocity command
        cmd_vel_msg.linear.x = 0.0
        self.cmd_vel_publisher.publish(cmd_vel_msg)

if __name__ == '__main__':
    navigator_demo = NavigatorDemo()

    # First goal pose
    goal_pose_1 = PoseStamped()
    goal_pose_1.header.frame_id = 'map'
    goal_pose_1.header.stamp = navigator_demo.navigator.get_clock().now().to_msg()
    goal_pose_1.pose.position.x =  5.43399
    goal_pose_1.pose.position.y = -0.060445
    goal_pose_1.pose.orientation.z = -0.69354
    goal_pose_1.pose.orientation.w = 0.720418

    navigator_demo.run_demo(goal_pose_1)

    # Move forward to under the shelf
    navigator_demo.move_robot()

    # Load the shelf
    navigator_demo.load_shelf()

    # Move backward
    navigator_demo.move_back()

    # Update footprint shape
    navigator_demo.update_footprint_size()

    # Second goal pose
    goal_pose_2 = PoseStamped()
    goal_pose_2.header.frame_id = 'map'
    goal_pose_2.header.stamp = navigator_demo.navigator.get_clock().now().to_msg()
    goal_pose_2.pose.position.x = 0.538482
    goal_pose_2.pose.position.y = -1.02962
    goal_pose_2.pose.orientation.z = -0.733563

    navigator_demo.run_demo(goal_pose_2)

    # Unload the shelf
    navigator_demo.unload_shelf()

    # Move backward away from the shelf
    navigator_demo.move_back()

    # Third goal pose
    shipping_goal = PoseStamped()
    shipping_goal.header.frame_id = 'map'
    shipping_goal.header.stamp = navigator_demo.navigator.get_clock().now().to_msg()
    shipping_goal.pose.position.x = 0.942255
    shipping_goal.pose.position.y = -3.25139
    shipping_goal.pose.orientation.z = -0.99977
    shipping_goal.pose.orientation.w = 0.0214511

    navigator_demo.run_demo(shipping_goal)

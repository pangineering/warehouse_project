#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose as NavigateToPoseAction
import tf2_ros
from sensor_msgs.msg import LaserScan
import math
from geometry_msgs.msg import TransformStamped, Twist
import time
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped

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

        self.robot_x = 0.0  # Initialize robot's x position
        self.robot_y = 0.0  # Initialize robot's y position
        self.initial_pose = None  # Variable to store initial pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10  # Adjust the queue size as needed
        )
        self.leg1_x = 0.0
        self.leg1_y = 0.0
        self.leg2_x = 0.0
        self.leg2_y = 0.0
        self.laser_detected_legs = False
        self.latest_scan = None
        self.intensity_threshold = 0.5  # Adjust threshold as needed


    def laser_callback(self, msg):
        # Store the received laser scan message
        self.latest_scan = msg

        # Implement logic to analyze laser intensity values and set
        # laser_detected_legs accordingly
        # Example: if laser intensity values indicate legs are detected:
        # self.laser_detected_legs = True
        # else:
        # self.laser_detected_legs = False

        # Placeholder logic for demonstration purposes (replace with actual logic)
        self.laser_detected_legs = self.detect_shelf_legs()


    def execute(self):


        print("Going to loading position")
        # Send the first goal to the robot to reach the loading_position
        loading_goal = PoseStamped()
        loading_goal.pose.position.x = 5.43399
        loading_goal.pose.position.y = -0.060445
        loading_goal.pose.position.z = 0.0
        loading_goal.pose.orientation.x = 0.0
        loading_goal.pose.orientation.y = 0.0
        loading_goal.pose.orientation.z = -0.697859
        loading_goal.pose.orientation.w = 0.716235
        self.send_goal(loading_goal)


        # Wait for the goal to be achieved
        while self._action_client.result.status != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(self)

        print("reached the loading position")
        current_pose = self.get_current_robot_pose()
        if current_pose:
            self.send_goal(current_pose)
        else:
            self.get_logger().warn("Could not retrieve current pose. Sending a default goal...")

        self.move_under_shelf()
        # Activate the elevator and move the shelf using the code from Checkpoint 5
        self.load_shelf()


        # Send the robot to the shipping_position using a new goal
        shipping_goal = PoseStamped()
        shipping_goal.pose.position.x = 1.27959
        shipping_goal.pose.position.y = -2.40654
        shipping_goal.pose.position.z = 0.0
        shipping_goal.pose.orientation.x = 0.0
        shipping_goal.pose.orientation.y = 0.0
        shipping_goal.pose.orientation.z = 0.999864
        shipping_goal.pose.orientation.w = 0.0164729
        self.send_goal(shipping_goal)

        # Wait for the goal to be achieved
        while self._action_client.result.status != GoalStatus.STATUS_SUCCEEDED:
            rclpy.spin_once(self)

        # Move down the elevator
        self.move_down_elevator()


  

    def send_goal(self, goal_pose):
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('FEEDBACK: ' + str(feedback))

    def move_under_shelf(self):
        self.perform_final_approach_logic()
    
    def load_shelf(self):
        # Trigger the elevator up action by publishing a message to /elevator_up
        elevator_up_msg = Empty()
        self.elevator_up_publisher.publish(elevator_up_msg)
        self.get_logger().info("Triggered elevator up action.")

    def unload_shelf(self):
        pass

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

            forward_distance = 0.3

            for _ in range(4):
                self.move_towards_transform(cart_transform.transform.translation.x, cart_transform.transform.translation.y)

            while forward_distance > 0:
                step_distance = min(0.1, forward_distance)

                forward_cmd_vel_msg = Twist()
                forward_cmd_vel_msg.linear.x = 0.1

                self.cmd_vel_publisher.publish(forward_cmd_vel_msg)

                forward_distance -= step_distance
                time.sleep(0.1)

            stop_cmd_vel_msg = Twist()
            stop_cmd_vel_msg.linear.x = 0.0
            self.cmd_vel_publisher.publish(stop_cmd_vel_msg)

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


def main(args=None):
    rclpy.init(args=args)
    move_shelf_to_ship = MoveShelfToShip()
    move_shelf_to_ship.execute()
    rclpy.spin(move_shelf_to_ship)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.action import NavigateToPose

class MoveShelfToShip(Node):
    def __init__(self):
        super().__init__('move_shelf_to_ship')
        self.navigate_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def execute(self):
        # Localize the robot in init_position using Simple Commander API

        # Send the first goal to the robot to reach the loading_position
        loading_goal = PoseStamped()
        # Set the loading_position goal
        # loading_goal.pose = ...

        # Send the goal using the NavigateToPose action
        self.navigate_action_client.send_goal(loading_goal)
        self.navigate_action_client.wait_for_result()

        # Activate the elevator and move the shelf using the code from Checkpoint 5

        # Update the robot shape in Nav2 if needed

        # Send the robot to the shipping_position using a new goal

        # Move down the elevator

        # Restore the original robot shape parameter for navigation

        # Send another goal to the init_position

def main(args=None):
    rclpy.init(args=args)
    move_shelf_to_ship = MoveShelfToShip()
    move_shelf_to_ship.execute()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

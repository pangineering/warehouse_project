#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped, Twist, PolygonStamped, Point32
from std_msgs.msg import Empty
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import time
"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()
    loaded = False
    publisher_local_costmap = navigator.create_publisher(PolygonStamped, '/local_costmap/published_footprint', 10)
    
    # Create publishers for cmd_vel and elevator_up
    publisher_cmd_vel = navigator.create_publisher(Twist, '/robot/cmd_vel', 10)
    publisher_elevator_up = navigator.create_publisher(Empty, '/elevator_up', 10)
    publisher_elevator_down = navigator.create_publisher(Empty, '/elevator_down', 10)


    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Change from goal_pose to loading_pose
    loading_pose = PoseStamped()
    loading_pose.header.frame_id = 'map'
    loading_pose.header.stamp = navigator.get_clock().now().to_msg()
    loading_pose.pose.position.x = 4.20563  # Replace with the x-coordinate from the log
    loading_pose.pose.position.y =  0.310323  # Replace with the y-coordinate from the log
    loading_pose.pose.orientation.z =  -0.698828 # Replace with the z-coordinate from the log
    loading_pose.pose.orientation.w = 0.71529  # Replace with the w-coordinate from the log

    # ...

    # Use the loading_pose variable later in your code
    navigator.goToPose(loading_pose)

    # ...

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')
    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
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
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            #if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #    goal_pose.pose.position.x = -3.0
            #    navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        if not loaded:
            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = 0.1  # Adjust linear velocity as needed
            cmd_vel_msg.angular.z = 0.0  # No rotation

            # Publish the Twist message to control the robot's movement
            publisher_cmd_vel.publish(cmd_vel_msg)

            # Wait for the robot to reach the target position (using time.sleep)
            time.sleep(2)  # Example sleep duration (2.8 seconds)


            cmd_vel_msg.linear.x = 0.0  # Adjust linear velocity as needed
            cmd_vel_msg.angular.z = 0.0  # No rotation

            # Publish the Twist message to control the robot's movement
            publisher_cmd_vel.publish(cmd_vel_msg)


            elevator_up_msg = Empty()
            publisher_elevator_up.publish(elevator_up_msg)


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
            publisher_local_costmap.publish(msg)

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = -0.1  # Adjust linear velocity as needed
            cmd_vel_msg.angular.z = 0.0  # No rotation

            # Publish the Twist message to control the robot's movement
            publisher_cmd_vel.publish(cmd_vel_msg)

            # Wait for the robot to reach the target position (using time.sleep)
            time.sleep(2)  # Example sleep duration (2.8 seconds)


            cmd_vel_msg.linear.x = 0.0  # Adjust linear velocity as needed
            cmd_vel_msg.angular.z = 0.0  # No rotation

            # Publish the Twist message to controlhe robot' ts movement
            publisher_cmd_vel.publish(cmd_vel_msg)
            loaded = True
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()







    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 5.54641  # Replace with the x-coordinate from the log
    goal_pose.pose.position.y = -0.091232  # Replace with the y-coordinate from the log
    #goal_pose.pose.orientation.z = -0.69354  # Replace with the z-coordinate from the log
    #goal_pose.pose.orientation.w = 0.720418  # Replace with the w-coordinate from the log

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
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
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            #if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
            #    goal_pose.pose.position.x = -3.0
            #    navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    if loaded:
        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.538482  # Replace with the x-coordinate from the log
        goal_pose.pose.position.y = -1.02962  # Replace with the y-coordinate from the log
        goal_pose.pose.orientation.z = -0.733563  # Replace with the z-coordinate from the log
        goal_pose.pose.orientation.w = 0.679621  # Replace with the w-coordinate from the log

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
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
                    navigator.cancelTask()

                # Some navigation request change to demo preemption
                #if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                #    goal_pose.pose.position.x = -3.0
                #    navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

        # Go to our demos first goal pose
        shipping_goal = PoseStamped()
        shipping_goal.header.frame_id = 'map'
        shipping_goal.header.stamp = navigator.get_clock().now().to_msg()
        shipping_goal.pose.position.x = 0.534934  # Replace with the x-coordinate from the log
        shipping_goal.pose.position.y = -2.13826  # Replace with the y-coordinate from the log
        shipping_goal.pose.orientation.z = 0.999998  # Replace with the z-coordinate from the log
        shipping_goal.pose.orientation.w =  0.00219527  # Replace with the w-coordinate from the log

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        navigator.goToPose(shipping_goal)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
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
                    navigator.cancelTask()

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')

            elevator_down_msg = Empty()
            publisher_elevator_down.publish(elevator_down_msg)


            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = -0.1  # Adjust linear velocity as needed
            cmd_vel_msg.angular.z = 0.0  # No rotation

            # Publish the Twist message to control the robot's movement
            publisher_cmd_vel.publish(cmd_vel_msg)

            # Wait for the robot to reach the target position (using time.sleep)
            time.sleep(2)  # Example sleep duration (2.8 seconds)


            cmd_vel_msg.linear.x = 0.0  # Adjust linear velocity as needed
            cmd_vel_msg.angular.z = 0.0  # No rotation

            # Publish the Twist message to control the robot's movement
            publisher_cmd_vel.publish(cmd_vel_msg)
            loaded = False




        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    

    if not loaded:
        init_pose = PoseStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = navigator.get_clock().now().to_msg()
        init_pose.pose.position.x = 0.0705415  # Replace with the x-coordinate from the log
        init_pose.pose.position.y =  -0.00758749  # Replace with the y-coordinate from the log
        init_pose.pose.orientation.z = 0.0230301  # Replace with the z-coordinate from the log
        init_pose.pose.orientation.w =  0.999735  # Replace with the w-coordinate from the log

        # sanity check a valid path exists
        # path = navigator.getPath(init_pose, goal_pose)

        navigator.goToPose(init_pose)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()
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
                    navigator.cancelTask()

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    #navigator.lifecycleShutdown()

    #exit(0)


if __name__ == '__main__':
    main()
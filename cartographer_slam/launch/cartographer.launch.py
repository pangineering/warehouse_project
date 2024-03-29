# cartographer.launch.py
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Find the path to the Cartographer configuration directory
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')

    return LaunchDescription([
        # Declare an argument for the Cartographer configuration file path
        DeclareLaunchArgument(
            'cartographer_config',
            default_value=os.path.join(cartographer_config_dir, 'cartographer.lua'),
            description='Path to the Cartographer configuration file',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/user/ros2_ws/src/warehouse_project/cartographer_slam/config/mapping.rviz'],
            parameters=[{'use_sim_time': False}]
        ),
        # Launch the Cartographer node
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', 'cartographer.lua']
,
        ),
        # Launch the OccupancyGrid node
        Node(
            package='cartographer_ros',
            executable='occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': False}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '0.5']
,
        ),
    ])

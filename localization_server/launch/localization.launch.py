import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
    # Declare a launch argument for 'map_file'
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml',  # Default value, change as needed
        description='Map file name (e.g., warehouse_map_sim.yaml)'
    )


    map_file_param = LaunchConfiguration('map_file')
    return LaunchDescription([
        declare_map_file_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/user/ros2_ws/src/warehouse_project/localization_server/config/localization.rviz'],
            parameters=[{'use_sim_time': False}]
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'yaml_filename': [
                    get_package_share_directory('map_server'),
                    '/config/',
                    map_file_param
                ],
            }],
        ),

            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])

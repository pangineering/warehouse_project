import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for 'map_file'
    declare_map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(
            get_package_share_directory('map_server'),
            'config',
            'warehouse_map_sim.yaml'
        ),
        description='Full path to the map file (e.g., warehouse_map_sim.yaml)'
    )

    map_file_param = LaunchConfiguration('map_file')

    return LaunchDescription([
        declare_map_file_arg,

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename': map_file_param}],

        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}],
  
        )
    ])

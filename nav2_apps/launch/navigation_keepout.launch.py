import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    costmap_filters_demo_dir = get_package_share_directory('nav2_apps')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(costmap_filters_demo_dir, 'config', 'filters.yaml'),
        description='Full path to the ROS 2 parameters file to use'
    )

    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
        'mask',
        default_value=os.path.join(costmap_filters_demo_dir, 'config', 'warehouse_map_sim_keepout.yaml'),
        description='Full path to filter mask yaml file to load'
    )

    param_substitutions = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'yaml_filename': LaunchConfiguration('mask')
    }

    configured_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key=LaunchConfiguration('namespace'),
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
            declare_params_file_cmd,
        declare_mask_yaml_file_cmd,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for costmap_filter_info_server'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/user/ros2_ws/src/warehouse_project/path_planner_server/rviz/pathplanning.rviz'],
            parameters=[{'use_sim_time': True}]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[configured_params]),



        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            parameters=[configured_params]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=["/home/user/ros2_ws/src/warehouse_project/path_planner_server/config/planner_server.yaml"]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            remappings=[('/cmd_vel', '/robot/cmd_vel')],
            parameters=["/home/user/ros2_ws/src/warehouse_project/path_planner_server/config/controller.yaml"]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=["/home/user/ros2_ws/src/warehouse_project/path_planner_server/config/bt.yaml"]
        ),
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            parameters=["/home/user/ros2_ws/src/warehouse_project/path_planner_server/config/recovery.yaml"],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['filter_mask_server','costmap_filter_info_server','planner_server', 'controller_server', 'bt_navigator', 'recoveries_server']}],
        ),



    ])

if __name__ == '__main__':
    generate_launch_description()

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'rviz', 'nav.rviz')
    nav2_yaml = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'config', 'amcl_config.yaml')
    map_file = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'maps', 'small_apartment2.yaml')

    controller_yaml = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory(
        'fastbot_slam'), 'config', 'recovery.yaml')

    return LaunchDescription([

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file}]
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml],
            remappings=[('/cmd_vel', '/fastbot/cmd_vel'),
                        ('/odom', '/fastbot/odom')]
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen',
            remappings=[('/cmd_vel', '/fastbot/cmd_vel'),
                        ('/odom', '/fastbot/odom')]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'planner_server',
                                        'controller_server',
                                        'recoveries_server',
                                        'bt_navigator']}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])

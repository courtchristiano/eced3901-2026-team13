#! /usr/bin/env python3
# Author: Addison Sears-Collins
# Modified: V. Sieben & E. Quill, Mar 2026
# Launch file for simulation with your part3.py waypoint follower

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    # Paths to package files
    pkg_share = FindPackageShare(package='eced3901').find('eced3901')
    default_launch_dir = os.path.join(pkg_share, 'launch')
    default_model_path = os.path.join(pkg_share, 'models/eced3901bot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2.rviz')
    static_map_path = os.path.join(pkg_share, 'maps', 'lab4_map.yaml')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch configuration variables
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_yaml_file = LaunchConfiguration('map')
    model = LaunchConfiguration('model')
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    slam = LaunchConfiguration('slam')
    use_namespace = LaunchConfiguration('use_namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace', default_value='False', description='Whether to apply namespace')
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Automatically startup nav2')
    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename', default_value=behavior_tree_xml_path, description='Behavior tree XML file')
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=static_map_path, description='Map file to load')
    declare_model_path_cmd = DeclareLaunchArgument(
        'model', default_value=default_model_path, description='URDF file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=nav2_params_path, description='Nav2 params file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file', default_value=default_rviz_config_path, description='RViz config file')
    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether to run SLAM')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True', description='Whether to start RViz')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use sim time')

    # RViz node
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Launch ROS2 Navigation
    start_ros2_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': slam,
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': autostart
        }.items()
    )

    # Launch your waypoint follower (part3.py)
    start_wp_follower = Node(
        package='eced3901',
        executable='part3',  # your updated waypoint follower
        name='part3',
        output='screen'
    )

    # Create launch description and add actions
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_ros2_navigation_cmd)
    ld.add_action(start_wp_follower)

    return ld

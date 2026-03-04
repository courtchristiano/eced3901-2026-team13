# part2.py
# Launch file to run SLAM + part1 mapping node
# Author: Adapted from Addison Sears-Collins & V. Sieben
# Date: 2026

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Paths
    pkg_share = FindPackageShare(package='eced3901').find('eced3901')
    default_launch_dir = os.path.join(pkg_share, 'launch')
    default_model_path = os.path.join(pkg_share, 'models/eced3901bot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_config.rviz')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    static_map_path = os.path.join(pkg_share, 'maps', 'part2_map')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_yaml_file = LaunchConfiguration('map')
    model = LaunchConfiguration('model')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    slam = LaunchConfiguration('slam')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Remappings (tf)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Declare Launch Arguments
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument('use_namespace', default_value='False', description='Apply namespace to nav stack')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup nav2 stack')
    declare_bt_xml_cmd = DeclareLaunchArgument('default_bt_xml_filename', default_value=behavior_tree_xml_path, description='Behavior tree XML')
    declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=static_map_path, description='Map YAML file to load')
    declare_model_path_cmd = DeclareLaunchArgument('model', default_value=default_model_path, description='Robot URDF path')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=nav2_params_path, description='ROS2 params file for nodes')
    declare_rviz_config_file_cmd = DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config_path, description='RViz config file')
    declare_slam_cmd = DeclareLaunchArgument('slam', default_value='True', description='Run SLAM')
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='True', description='Start RViz')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock if true')

    # Nodes
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

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

    # Run your mapping node (part1.cpp)
    start_part1 = Node(
        package='eced3901',
        executable='part1',
        name='part1_mapper',
        output='screen'
    )

    # Delay mapping start a few seconds so nav stack initializes
    delayed_part1 = TimerAction(period=5.0, actions=[start_part1])

    # Map saving node
    start_mapsave = Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        name='map_saver',
        output='screen',
        arguments=['-f', map_yaml_file]
    )

    delayed_mapsave = TimerAction(period=80.0, actions=[start_mapsave])

    # Launch description
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
    ld.add_action(delayed_part1)
    ld.add_action(delayed_mapsave)

    return ld

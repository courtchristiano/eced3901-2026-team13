# Author: Addison Sears-Collins
# Modified: E. Quill, Mar 2026
# Launch file to start Gazebo, Nav2 with your saved map, RViz, and waypoint follower

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Package paths
    pkg_share = FindPackageShare(package='eced3901').find('eced3901')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch')

    # Paths to files
    default_model_path = os.path.join(pkg_share, 'models/eced3901bot.urdf')
    rviz_config_path = os.path.join(pkg_share, 'rviz/nav2.rviz')
    map_path = os.path.join(pkg_share, 'maps', 'part2_map.yaml')  # ✅ your map
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    declare_use_namespace_cmd = DeclareLaunchArgument('use_namespace', default_value='False', description='Whether to apply a namespace to the navigation stack')
    declare_autostart_cmd = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup the nav2 stack')
    declare_bt_xml_cmd = DeclareLaunchArgument('default_bt_xml_filename', default_value=behavior_tree_xml_path, description='Behavior tree XML file')
    declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=map_path, description='Full path to map file')
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=nav2_params_path, description='ROS2 parameters file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument('rviz_config_file', default_value=rviz_config_path, description='Full path to the RVIZ config file')
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='True', description='Start RViz')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use simulation clock')

    # Start RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Start Nav2 stack
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'slam': 'False',
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_bt_xml_filename': default_bt_xml_filename,
            'autostart': autostart
        }.items()
    )

    # Start your waypoint follower
    start_wp_follower = Node(
        package='eced3901',
        executable='part3.py',  # ✅ your waypoint follower script
        name='part3',
        output='screen'
    )

    # Create LaunchDescription and add actions
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add nodes
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_nav2_cmd)
    ld.add_action(start_wp_follower)

    return ld

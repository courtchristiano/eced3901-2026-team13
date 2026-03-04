import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Paths to package and files
    pkg_share = FindPackageShare(package='eced3901').find('eced3901')
    default_launch_dir = os.path.join(pkg_share, 'launch')
    default_model_path = os.path.join(pkg_share, 'models/eced3901bot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2.rviz')
    static_map_path = os.path.join(pkg_share, 'maps', 'part2_map.yaml')  # your saved map
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')

    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
    nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')

    # Launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace', default='False')
    autostart = LaunchConfiguration('autostart', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='True')
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename', default=behavior_tree_xml_path)
    map_yaml_file = LaunchConfiguration('map', default=static_map_path)
    params_file = LaunchConfiguration('params_file', default=nav2_params_path)
    rviz_config_file = LaunchConfiguration('rviz_config_file', default=default_rviz_config_path)

    # Declare launch arguments
    declare_map_cmd = DeclareLaunchArgument('map', default_value=static_map_path, description='Full path to map file')
    declare_use_rviz_cmd = DeclareLaunchArgument('use_rviz', default_value='True', description='Launch RViz')
    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='True', description='Use sim clock')

    # Launch RViz
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
        launch_arguments = {
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

    # Launch your part3.py waypoint follower after 10s delay
    from launch.actions import TimerAction
    start_part3 = Node(
        package='eced3901',
        executable='part3.py',  # updated executable
        name='part3',
        output='screen'
    )
    delayed_part3 = TimerAction(period=10.0, actions=[start_part3])

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_ros2_navigation_cmd)
    ld.add_action(delayed_part3)

    return ld

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    pkg_share = FindPackageShare(package='eced3901').find('eced3901')
    nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup')

    # Paths
    default_model_path = os.path.join(pkg_share, 'models/eced3901bot.urdf')
    static_map_path = os.path.join(pkg_share, 'maps', 'part4map.yaml')
    nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
    nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
    behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2.rviz')

    nav2_launch_dir = os.path.join(nav2_dir, 'launch')

    # Launch arguments
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')

    # Declare launch arguments
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('map', default_value=static_map_path))
    ld.add_action(DeclareLaunchArgument('params_file', default_value=nav2_params_path))
    ld.add_action(DeclareLaunchArgument('default_bt_xml_filename', default_value=behavior_tree_xml_path))
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True'))
    ld.add_action(DeclareLaunchArgument('autostart', default_value='True'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='True'))
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument('use_namespace', default_value='False'))
    ld.add_action(DeclareLaunchArgument('slam', default_value='False'))

    # Start Gazebo with empty world (replace with a world matching your map if available)
    gazebo_world = os.path.join(pkg_share, 'worlds', 'empty.world')
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gazebo_ros').find('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': gazebo_world}.items()
    )
    ld.add_action(start_gazebo)

    # Spawn robot at 0,0 facing +Y
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'eced3901bot',
            '-file', default_model_path,
            '-x', '0',
            '-y', '0',
            '-Y', '1.57'  # facing +Y
        ],
        output='screen'
    )
    ld.add_action(spawn_robot)

    # Start Nav2
    start_nav2 = IncludeLaunchDescription(
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
    ld.add_action(start_nav2)

    # Start RViz
    start_rviz = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_config_path]
    )
    ld.add_action(start_rviz)

    # Start your waypoint follower (part3.py)
    start_wpfollow = Node(
        package='eced3901',
        executable=os.path.join(pkg_share, 'scripts', 'part3.py'),
        name='part3',
        output='screen'
    )
    ld.add_action(start_wpfollow)

    return ld

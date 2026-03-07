#!/usr/bin/env python3

import os
import time
import rclpy
from rclpy.node import Node
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

import subprocess
import signal
import numpy as np

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def launch_nav2(map_path):
    """Launch Nav2 bringup_launch.py as a subprocess"""
    nav2_pkg_share = FindPackageShare('nav2_bringup').find('nav2_bringup')
    bringup_launch = os.path.join(nav2_pkg_share, 'launch', 'bringup_launch.py')

    cmd = [
        'ros2', 'launch', bringup_launch,
        f'map:={map_path}',
        'autostart:=True',
        'use_sim_time:=False'
    ]

    print(f"Launching Nav2: {cmd}")
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, preexec_fn=os.setsid)

def main():
    rclpy.init()

    # Launch Nav2 automatically
    map_file = os.path.expanduser('~/ros2_ws/src/eced3901/maps/part4map.yaml')
    nav2_process = launch_nav2(map_file)

    # Allow Nav2 to initialize
    print("Waiting 10-15 seconds for Nav2 to start...")
    time.sleep(15)

    navigator = BasicNavigator()
    print("Waiting for Nav2 to activate...")
    navigator.waitUntilNav2Active()

    # Define waypoints (x, y, yaw)
    waypoints_coords = [
        [0.0, 1.829, 1.5708],  # 6 ft along y
        [0.0, 2.438, 1.5708],  # 8 ft (after wall)
        [0.0, 3.353, 1.5708]   # 11 ft (port)
    ]

    poses = []
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = 'map'

    for wp in waypoints_coords:
        pose_msg.header.stamp = navigator.get_clock().now().to_msg()
        pose_msg.pose.position.x = wp[0]
        pose_msg.pose.position.y = wp[1]
        q = get_quaternion_from_euler(0, 0, wp[2])
        pose_msg.pose.orientation.x = q[0]
        pose_msg.pose.orientation.y = q[1]
        pose_msg.pose.orientation.z = q[2]
        pose_msg.pose.orientation.w = q[3]
        poses.append(deepcopy(pose_msg))

    print("Sending waypoints...")
    navigator.followWaypoints(poses)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"Waypoint {feedback.current_waypoint+1}/{len(poses)}")
        time.sleep(0.5)

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Navigation complete!")
    else:
        print("Navigation failed or canceled!")

    # Shutdown Nav2 process
    print("Shutting down Nav2...")
    os.killpg(os.getpgid(nav2_process.pid), signal.SIGTERM)

    navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


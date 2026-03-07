#!/usr/bin/env python3
import rclpy
from copy import deepcopy
import numpy as np

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def get_quaternion_from_euler(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

def main():
    rclpy.init()
    navigator = BasicNavigator()

    # Waypoints along Y-axis in meters
    waypoints = [
        [0.0, 1.8288, 1.57],   # 6 ft up (+Y)
        [0.0, 2.4384, 1.57],   # 8 ft up, just after wall
        [0.0, 3.3528, 1.57]    # 11 ft up, port
    ]

    # Set initial pose at origin facing +Y
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    q = get_quaternion_from_euler(0, 0, 1.57)
    initial_pose.pose.orientation.x = q[0]
    initial_pose.pose.orientation.y = q[1]
    initial_pose.pose.orientation.z = q[2]
    initial_pose.pose.orientation.w = q[3]
    navigator.setInitialPose(initial_pose)

    # Wait until Nav2 is active
    navigator.waitUntilNav2Active()

    # Build PoseStamped messages for waypoints
    waypoint_msgs = []
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    for pt in waypoints:
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        q = get_quaternion_from_euler(0, 0, pt[2])
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        waypoint_msgs.append(deepcopy(pose))

    # Send robot along waypoints
    navigator.followWaypoints(waypoint_msgs)

    # Monitor progress
    step_counter = 0
    while not navigator.isTaskComplete():
        step_counter += 1
        feedback = navigator.getFeedback()
        if feedback and step_counter % 5 == 0:
            print(f"Executing waypoint {feedback.current_waypoint + 1}/{len(waypoint_msgs)}")

    # Check result
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Demo complete! Robot reached port.")
    elif result == TaskResult.CANCELED:
        print("Demo canceled!")
    else:
        print("Demo failed!")

    # Optionally return to start
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    navigator.goToPose(initial_pose)
    while not navigator.isTaskComplete():
        pass

    exit(0)

if __name__ == '__main__':
    main()

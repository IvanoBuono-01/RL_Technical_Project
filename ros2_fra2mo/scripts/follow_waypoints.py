#!/usr/bin/env python3
# Copyright 2021 Samsung Research America
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import yaml

waypoints = yaml.safe_load('''
waypoints:
  - position:
      x: 16.2
      y: -7.2
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 0.0
''')

print(f"Caricati {len(waypoints['waypoints'])} waypoints")

def main():
    rclpy.init()
    navigator = BasicNavigator()

    def create_pose(transform):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        pose.pose.orientation.x = transform["orientation"]["x"]
        pose.pose.orientation.y = transform["orientation"]["y"]
        pose.pose.orientation.z = transform["orientation"]["z"]
        pose.pose.orientation.w = transform["orientation"]["w"]
        
        print(f"Waypoint: ({pose.pose.position.x:.1f}, {pose.pose.position.y:.1f})")
        return pose

    goal_poses = list(map(create_pose, waypoints["waypoints"]))
    print(f"{len(goal_poses)} waypoints pronti")

    navigator.waitUntilNav2Active(localizer="smoother_server")
    print("Nav2 attivo")

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)
    print("Navigazione avviata")

    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        
        if feedback and i % 5 == 0:
            print(f"Waypoint {feedback.current_waypoint + 1}/{len(goal_poses)}")
        
        now = navigator.get_clock().now()
        if now - nav_start > Duration(seconds=600):
            print("Timeout raggiunto")
            navigator.cancelTask()
            break

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    
    print("Script terminato")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
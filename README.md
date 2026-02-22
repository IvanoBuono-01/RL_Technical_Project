# Robotic Warehouse - Robotics Lab 2025

**Author**: Buono Ivano (Matr: P38000332)

## Project Overview

This repository contains all the files necessay to run the final technical project fot the **Robot Lab 2025** course.
The system simulates a **multi-robot collaborative environment** build using **ROS 2**, **RVIZ** and **Gazebo**.

The project includes:

- **Fra2mo** : a differential-drive robot equipped with *LiDAR* to make possibile the autonomous navigation and mapping;

-**KUKA IIWA** : a 7 DoFs industrial manipulator used for item grasping and positioning.

The goal of this simulation is to exectute a list of singles tasks: navigation to a pre-speficied goal (accomplished by Fra2mo) and pick & place of the items (accomplished by the KUKA IIWA).

## Getting Started

First clone the repository inside ROS2, build and source the environment:

```shell
cd ~/ros2_ws/src
git clone https://github.com/IvanoBuono-01/RL_Technical_Project.git
colcon build
source install/setup.bash
```

## Exploration and Mapping

### Exploration

First we need to start the autonomous exploration of the unknown environment:

```shell
ros2 launch ros2_fra2mo fra2mo_explore.launch.py    
```

After the exploration is complete it's necessary to save the generated map:

After the exploration is complete, save the generated map:
```shell
cd ~/ros2_ws/src/ros2_fra2mo/maps/
ros2 run nav2_map_server map_saver_cli -f personal_project_world
```
## Running the simulation

To launch the world for the Autonomous Warehouse:

```shell
ros2 launch deposit spawn_deposit.launch.py
```

### Running the localization for the Fra2mo

```shell
ros2 launch ros2_fra2mo fra2mo_navigation.launch.py
```

### Starting the navigation to goal 

```shell
ros2 run ros2_fra2mo follow_waypoints.py
```

### Exectuing the pick & placke

Once Fra2mo ended its task, to start the pick & place task

```shell
ros2 run deposit pick_package
```

## File Structure

```
├── deposit/                # Main logic node (pick & place), Launch files, World
├── ros2_fra2mo/            # Mobile robot description, Nav2 config, SLAM
├── ros2_iiwa/              # KUKA manipulator description and control
├── aruco_ros/              # Vision and marker detection packages
└── m-explore-ros2/         # Autonomous frontier exploration
```



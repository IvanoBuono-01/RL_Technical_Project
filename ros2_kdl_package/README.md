# ros2_kdl_package

## :package: About

This package contains the code to create and run the HOMEWORK2 of RoboticLab_2025

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace. Check for missing dependencies
```
rosdep install -i --from-path src --rosdistro humble -y
```
Build your new package
```
colcon build --packages-select ros2_kdl_package
```
Source the setup files
```
. install/setup.bash
```

## :white_check_mark: Usage Homework-Point-1
Run the  kdl_node.launch.py :
```
ros2 launch ros2_kdl_package kdl_node.launch.py
```

By default the node publishes joint position commands. 
To use the velocity commands (and specify the parameter ctrl:=velocity_ctrl or ctrl:=velocity_ctrl_null) : 
```
ros2 launch ros2_kdl_package kdl_node.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl
```
or the other velocity controller : 
```
ros2 launch ros2_kdl_package kdl_node.launch.py cmd_interface:=velocity ctrl:=velocity_ctrl_null
```

in this two case the robot must be launched with the velocity interface in another terminal: 
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

Now in the another terminal launch the action client to the our code : 
```
ros2 run ros2_kdl_package trajectory_action_client --ros-args --params-file src/ros2_kdl_package/config/kdl_params.yaml
```

## :white_check_mark: Usage Homework-Point-2

The robot must be launched with the velocity interface and the parameter use_sim to open gazebo :
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller" use_sim:="true"
```

In another terminal launch this command to use a new vison controller (ctrl:=vision) : 
```
ros2 launch ros2_kdl_package kdl_node.launch.py cmd_interface:=velocity ctrl:=vision
```
Homework-Point-2b: if you want to manually move the aruco_tag in the gazebo you will see the tracking capability of the controller.

Homework-Point-2c: but if you want to use a ROS 2 service to update the aruco marker position in Gazebo use this ROS 2 service call in another terminal :
```
ros2 service call /world/iiwa_personal_world/set_pose ros_gz_interfaces/srv/SetEntityPose "{
  entity: {name: 'aruco_tag', id: 0, type: 2}, 
  pose: {position: {x: -0.159, y: -0.600, z: 0.49}, orientation: {x: -0.59, y: 0.4, z: -.4, w: 0.58}}
}"
```












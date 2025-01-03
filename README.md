# LLM_P3_ROS2

## Overview

This project is a ROS2 package that simulates a sjtu drone robot in Gazebo controlled by OpenAI APIs. The robot is equipped with a camera and a lidar. 

## Dependencies

This project was developed under Ubuntu 20.04 and ROS2 Foxy. It requires the following packages:

- ROS2 Humble
- Gazebo 11
- Rviz2
- OpenAI APIs

## Build Instructions

To build the project, create a workspace and clone the repository in it. Then, build the workspace with the following commands:

```
colcon build
```

## Run Instructions

To run the project, source the workspace and launch the launch file with the following commands:

``` 
sudo apt-get install ros-humble-joint-state-publisher
ros2 launch world.launch.py
```

## Launch

```
killall gzserver && killall gzclient
rm -rf build/ install/ log/ && colcon build
source install/setup.bash
source install/setup.bash && ros2 launch sjtu_drone_bringup sjtu_drone_tunnel.launch.py

ros2 launch sjtu_drone_bringup sjtu_drone_gazebo.launch.py
ros2 topic pub /drone/takeoff std_msgs/msg/Empty "{}" --once
ros2 topic pub /drone/land std_msgs/msg/Empty "{}" --once
```

## Author

This project was developed by [Boringtao](https://twitter.com/BoringtaoL22644).

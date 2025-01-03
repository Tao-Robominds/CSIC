# Gazebo Tunnel World

This is a [Gazebo](http://gazebosim.org/) world built by Wenbo Zhao from [CERLAB](http://www.andrew.cmu.edu/user/shimada/iam/) to simulate under-construction-tunnel environment. The world is designed to simulate UAV SLAM and planning algorithms. Part of the models are downloaded and modified from [3D Warehouse](https://3dwarehouse.sketchup.com/).

Before opening `tunnel_world.sdf` file in Gazebo, the models need to be imported to Gazebo model directory.
Command for adding the path to Gazebo model directory:
```
export GAZEBO_MODEL_PATH=$PATH:path/to/the/world/tunnel_models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix sjtu_drone_description)/worlds/tunnel/tunnel_models

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/boringtao/ROS2/CSIC/src/drone/sjtu_drone_description/worlds/tunnel/tunnel_models

cd ~/.gazebo/models
mkdir -p tunnel
cp -r /home/boringtao/ROS2/CSIC/src/drone/sjtu_drone_description/worlds/tunnel_models/* ~/.gazebo/models/
```

Then run:
```
gazebo $(ros2 pkg prefix sjtu_drone_description)/worlds/tunnel/tunnel_world.sdf
gazebo /home/boringtao/ROS2/CSIC/src/drone/sjtu_drone_description/worlds/tunnel_world.sdf
```

## Screenshot

![Screenshot-1](https://github.com/zwb55/Gazebo-Tunnel-World/blob/master/Screenshots/Screenshot_tunnel_world.png)

#!/bin/bash

export IGN_GAZEBO_RESOURCE_PATH="$(pwd)/models"
ign gazebo -r ./sem_$1/world.sdf &

source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$(pwd)/ros_gz_bridge.yaml"
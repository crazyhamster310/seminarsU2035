#!/bin/bash

export IGN_GAZEBO_RESOURCE_PATH="$(pwd)"
ign gazebo ./empty.sdf &
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$(pwd)/ros_gz_bridge.yaml"
#!/bin/bash

# source /opt/ros/humble/setup.bash

main_dir=$(pwd)
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args \
  -p config_file:="$main_dir/ros_gz_bridge.yaml" \
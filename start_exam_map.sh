#!/bin/bash

main_dir=$(pwd)

export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:"$main_dir/models"
export IGN_IP=127.0.0.1

ign gazebo -r "$main_dir/exam/world.sdf"

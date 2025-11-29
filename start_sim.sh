#!/bin/bash

if [ $# -ne 1 ]; then
  echo "Ошибка: укажи id среды" >&2
  exit 1
fi

if ! [[ "$1" =~ ^-?[0-9]+$ ]]; then
  echo "Ошибка: аргумент должен быть целым числом." >&2
  exit 1
fi

sem_num=$1
main_dir=$(pwd)

export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:"$main_dir/models"
export IGN_IP=127.0.0.1

ign gazebo -r "$main_dir/seminar$sem_num/world.sdf"

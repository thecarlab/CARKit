#!/bin/bash

source install/setup.bash

ros2 launch carkit_human_control joystick.launch.py &
sleep 3

ros2 launch carkit_navigation navigation.launch.py \
  map:=/workspaces/CARKit/map/map_5fs.yaml \
  visualization:=rviz &
sleep 5

ros2 run carkit_amcl simple_loop_controller --ros-args \
  -p turn_radius:=1.0 \
  -p straight_distance:=2.0 \
  -p ackermann_topic:=/ackermann_cmd

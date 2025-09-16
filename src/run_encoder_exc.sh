#!/bin/bash

sudo bash -c '
  source /opt/ros/humble/setup.bash
  source /home/skolek/robot/install/setup.bash

  echo "Using ros2 from: $(which ros2)"
  exec taskset -c 3 chrt -f 80 ros2 run sensors_pkg encoder_node
'

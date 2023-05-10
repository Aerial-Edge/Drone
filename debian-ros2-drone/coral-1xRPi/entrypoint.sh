#!/bin/bash

source install/setup.bash
sudo udevadm control --reload
(trap 'kill 0' SIGINT; ros2 run drone_pkg autopilot_node & ros2 run drone_controller controller_node & ros2 run img_processing videostream_node & ros2 run img_processing detect_node & wait)

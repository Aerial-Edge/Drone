#!/bin/bash

source install/setup.bash
(trap 'kill 0' SIGINT; ros2 run drone_pkg autopilot_node & ros2 run drone_controller controller_node & wait)

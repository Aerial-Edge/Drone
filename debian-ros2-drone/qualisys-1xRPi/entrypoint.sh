#!/bin/bash

source install/setup.bash
(trap 'kill 0' SIGINT; 	ros2 run drone_pkg autopilot_node & \
			ros2 run drone_controller controller_node & \
			ros2 run config4 camera_capture & \
			ros2 run config4 object_detection & \
                        ros2 run core follow_algorithm & \
                        ros2 run qtm_logger log & \
                        ros2 run qualisys_node qualisys & wait)

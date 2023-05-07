FROM docker.aerial-edge.no/debian_ros2_humble:1.0.0

# Installing pyMAVLink:
RUN pip install --extra-index-url=https://www.piwheels.org/simple numpy pymavlink

# Installing pynput:
RUN pip install pynput

# Initializing packages:
WORKDIR	/home/ros/ros2_ws/src/
RUN . /home/ros/ros2_ws/install/setup.sh && \
    ros2 pkg create --build-type ament_python --node-name autopilot_node drone_pkg && \
    ros2 pkg create --build-type ament_python drone_controller


# Copying package files:
COPY ./drone_pkg ./drone_pkg/
COPY ./drone_controller ./drone_controller/

# Building packages:
WORKDIR /home/ros/ros2_ws/
RUN . /home/ros/ros2_ws/install/setup.sh && \
    colcon build --packages-select drone_pkg drone_controller --symlink-install

# Copying entrypoint:
COPY ./entrypoint.sh ./

ENTRYPOINT ["./entrypoint.sh"]

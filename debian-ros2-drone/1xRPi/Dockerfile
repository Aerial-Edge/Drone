FROM docker.aerial-edge.no/debian_ros2_humble:1.0.0
WORKDIR ~/

# Installing NumPy & pyMAVLink:
RUN pip install --extra-index-url=https://www.piwheels.org/simple numpy pymavlink

# Installing pynput:
RUN pip install pynput

# Installing OpenCV:
RUN pip install opencv-contrib-python cvzone

# Downloading OpenCV bridge pkg for ROS2:
WORKDIR ~/ros2_ws/src/
RUN git clone https://github.com/ros-perception/vision_opencv.git --branch humble
WORKDIR  ~/ros2_ws/
RUN colcon build --packages-select vision_opencv

# Initialize packages:
WORKDIR ~/ros2_ws/src/
RUN . /home/ros/ros2_ws/install/setup.sh && \
    ros2 pkg create --build-type ament_python --node-name autopilot_node drone_pkg && \
    ros2 pkg create --build-type ament_python drone_controller && \
    ros2 pkg create --build-type ament_python config4

# Copying package files:
COPY ./src/ ./

# Building packages:
WORKDIR /home/ros/ros2_ws/
RUN . /home/ros/ros2_ws/install/setup.sh && \
    colcon build --packages-select drone_pkg drone_controller config4 --symlink-install

# Copying entrypoint:
COPY ./entrypoint.sh ./
RUN sudo chmod +x ./entrypoint.sh

RUN sudo apt-get update && sudo apt-get -y install python3-opencv
RUN sudo apt-get -y install python3-cv-bridge

ENTRYPOINT ["./entrypoint.sh"]

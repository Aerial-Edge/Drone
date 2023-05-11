FROM docker.aerial-edge.no/debian_ros2_humble:1.0.0
WORKDIR ~/

# Installing NumPy & pyMAVLink:
RUN pip install --extra-index-url=https://www.piwheels.org/simple numpy pymavlink

# Installing pynput:
RUN pip install pynput

# Installing CVZone:
RUN pip install cvzone

# Installing OpenCV:
USER root
RUN  apt-get update && apt-get -y install python3-opencv python3-cv-bridge

# Installing PiCamera2:
RUN mkdir /etc/apt/keyrings && \
    wget https://archive.raspberrypi.org/debian/raspberrypi.gpg.key && \
    gpg --no-default-keyring --keyring temp-keyring.gpg --import raspberrypi.gpg.key && \
    gpg --no-default-keyring --keyring temp-keyring.gpg --export --output /etc/apt/keyrings/raspberrypi-apt-key.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/raspberrypi-apt-key.gpg] http://archive.raspberrypi.org/debian/ bullseye main" | tee /etc/apt/sources.list.d/raspi.list && \
    rm raspberrypi.gpg.key && apt-get update && apt-get install -y python3-picamera2

# Enabling udev:
RUN apt-get update && apt-get -y install udev
ENV UDEV=on

# Setting up video permissions:
RUN usermod -aG video ros

# Initialize packages:
USER ros
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
    colcon build --packages-select drone_pkg drone_controller config4

# Copying entrypoint:
COPY ./entrypoint.sh ./
RUN sudo chmod +x ./entrypoint.sh


ENTRYPOINT ["./entrypoint.sh"]

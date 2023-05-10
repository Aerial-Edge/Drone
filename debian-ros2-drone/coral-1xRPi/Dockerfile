FROM docker.aerial-edge.no/debian_ros2_humble:1.0.0
WORKDIR ~/

# Installing NumPy & pyMAVLink:
RUN pip install --extra-index-url=https://www.piwheels.org/simple numpy pymavlink

# Installing pynput:
RUN pip install pynput

USER root
# Installing TensorFlow Lite:
RUN wget https://packages.cloud.google.com/apt/doc/apt-key.gpg && \
    gpg --no-default-keyring --keyring temp-keyring.gpg --import apt-key.gpg && \
    mkdir /etc/apt/keyrings/ && gpg --no-default-keyring --keyring temp-keyring.gpg --export --output /etc/apt/keyrings/google-apt-key.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/google-apt-key.gpg] https://packages.cloud.google.com/apt coral-edgetpu-stable main" | tee /etc/apt/sources.list.d/coral-edgetpu.list && \
    rm apt-key.gpg && apt-get update && apt-get -y install python3-tflite-runtime libedgetpu1-std python3-cv-bridge python3-opencv

# Installing PiCamera2:
RUN wget https://archive.raspberrypi.org/debian/raspberrypi.gpg.key && \
    gpg --no-default-keyring --keyring temp-keyring.gpg --import raspberrypi.gpg.key && \
    gpg --no-default-keyring --keyring temp-keyring.gpg --export --output /etc/apt/keyrings/raspberrypi-apt-key.gpg && \
    echo "deb [signed-by=/etc/apt/keyrings/raspberrypi-apt-key.gpg] http://archive.raspberrypi.org/debian/ bullseye main" | tee /etc/apt/sources.list.d/raspi.list && \
    rm raspberrypi.gpg.key && apt-get update && apt-get install -y python3-picamera2

# Setting up video permissions:
RUN usermod -aG video ros
RUN apt-get update && sudo apt-get -y install udev
ENV UDEV=on

# Initializing packages:
USER ros
WORKDIR	/home/ros/ros2_ws/src/
RUN . /home/ros/ros2_ws/install/setup.sh && \
    ros2 pkg create --build-type ament_python --node-name autopilot_node drone_pkg && \
    ros2 pkg create --build-type ament_python drone_controller && \
    ros2 pkg create --build-type ament_python img_processing

# Copying package files:
COPY ./src/ ./

# Building packages:
WORKDIR /home/ros/ros2_ws/
RUN . /home/ros/ros2_ws/install/setup.sh && \
    colcon build --packages-select drone_pkg drone_controller img_processing --symlink-install

# Copying entrypoint:
COPY ./entrypoint.sh ./
RUN sudo chmod +x ./entrypoint.sh

ENTRYPOINT ["./entrypoint.sh"]
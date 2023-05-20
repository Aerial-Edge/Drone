#FROM python:3.9-bullseye
FROM debian:bullseye-slim
WORKDIR /root/

# Installing Python3 PIP:
RUN apt-get update && apt-get -y install python3-pip

# Installing NumPy & Pandas:
RUN pip install --extra-index-url=https://www.piwheels.org/simple numpy pandas

# Installing TensorFlow Lite:
RUN apt-get -y install wget && \
    wget https://packages.cloud.google.com/apt/doc/apt-key.gpg && \
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
RUN apt-get update && apt-get -y install udev
ENV UDEV=on

# Copying source code:
COPY ./app ./app


CMD ["python3", "./app/detect_edl0.py"]

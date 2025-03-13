ARG BASE_IMAGE=ros:humble
FROM ${BASE_IMAGE} AS base
LABEL org.opencontainers.image.authors="ohin.kyuu@gmail.com"
LABEL org.opencontainers.image.vendor="DIT-Robotics"
ENV TERM=xterm-256color
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
    git \
    sudo \
    curl \
    wget \
    usbutils \
    v4l-utils \
    net-tools \
    iputils-ping \
    python3-pip \
    apt-transport-https \
    software-properties-common \  
    ros-humble-launch-pytest \
    # DDS
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rmw-fastrtps-cpp \
    # Camera packages
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-sensor-msgs-py \
    ros-humble-realsense2-camera-msgs \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \   
    && apt clean -y && rm -rf /var/lib/apt/lists/*

# Ceiling Camera Module
FROM base AS ceiling-cam
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN apt update && DEBIAN_FRONTEND=noninteractive apt install -y \
    gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-rtsp \
    ros-humble-yaml-cpp-vendor \
    ros-humble-camera-info-manager \
    ros-humble-image-pipeline \
    && apt clean -y && rm -rf /var/lib/apt/lists/*
RUN groupadd --gid $USER_GID $USER && \
    useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER && \
    echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
USER $USER
COPY packages/rtspcam-pkg /home/$USER/vision-ws/src/rtsp_ros
RUN sudo chmod -R 777 /home/$USER/vision-ws && \
    rosdep update && \
    rosdep install -i --from-path /home/$USER/vision-ws/src --rosdistro humble -y && \
    cd /home/$USER/vision-ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                 colcon build"
WORKDIR /home/$USER/vision-ws
CMD [ "/bin/bash" , "-c", "source /opt/ros/humble/setup.bash && source /home/ceiling-cam/vision-ws/install/local_setup.bash && ros2 launch rtsp_ros rtsp_cam.launch.py" ]


# Aruco Module
FROM base AS aruco
ARG USER
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USER && \
    useradd --uid $USER_UID --gid $USER_GID -ms /bin/bash $USER && \
    echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER && \
    echo "source /opt/ros/humble/setup.bash" >> /home/$USER/.bashrc
USER $USER
COPY packages/groundtruth-pkg /home/$USER/vision-ws/src/aruco-ros
RUN sudo chmod -R 777 /home/$USER/vision-ws && \
    rosdep update && \
    rosdep install -i --from-path /home/$USER/vision-ws/src --rosdistro humble -y && \
    cd /home/$USER/vision-ws && \
    /bin/bash -c "source /opt/ros/humble/setup.bash && \
                 colcon build"
WORKDIR /home/$USER/vision-ws
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /home/groundtruth/vision-ws/install/local_setup.bash && ros2 launch aruco_ros single.launch.py"]

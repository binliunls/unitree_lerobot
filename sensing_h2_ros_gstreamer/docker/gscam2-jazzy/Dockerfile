ARG BASE_IMAGE=arm64v8/ubuntu:24.04
ARG ROS_DISTRO=jazzy

FROM ${BASE_IMAGE}

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg \
    build-essential \
    cmake \
    pkg-config \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    v4l-utils \
    && rm -rf /var/lib/apt/lists/*

RUN install -d -m 0755 /etc/apt/keyrings \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /etc/apt/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
      > /etc/apt/sources.list.d/ros2.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ament-cmake \
    ros-${ROS_DISTRO}-camera-calibration-parsers \
    ros-${ROS_DISTRO}-camera-info-manager \
    ros-${ROS_DISTRO}-class-loader \
    ros-${ROS_DISTRO}-rclcpp \
    ros-${ROS_DISTRO}-rclcpp-components \
    ros-${ROS_DISTRO}-rclpy \
    ros-${ROS_DISTRO}-ros2run \
    ros-${ROS_DISTRO}-ros2topic \
    ros-${ROS_DISTRO}-sensor-msgs \
    python3-colcon-cmake \
    python3-colcon-core \
    python3-colcon-ros \
    && rm -rf /var/lib/apt/lists/*

# Do not install NVIDIA Jetson GStreamer packages here. On Jetson, the
# NVIDIA container runtime supplies nvvidconv and Jetson multimedia libraries
# from the host so they match the running JetPack / Jetson Linux release.
WORKDIR /opt/ros_ws/src
COPY gscam2 ./gscam2

WORKDIR /opt/ros_ws
RUN source "/opt/ros/${ROS_DISTRO}/setup.bash" \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log src/gscam2/.git

RUN printf '%s\n' \
    '#!/usr/bin/env bash' \
    'set -e' \
    'source "/opt/ros/${ROS_DISTRO}/setup.bash"' \
    'source /opt/ros_ws/install/setup.sh' \
    'exec "$@"' \
    > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

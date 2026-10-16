FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    python3-opencv \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

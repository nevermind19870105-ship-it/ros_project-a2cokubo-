FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install opencv-python numpy matplotlib

WORKDIR /workspace

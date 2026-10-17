FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y \
    git \
    patch \
    libtbb-dev \
    libboost-dev \
    libeigen3-dev \
    libassimp-dev \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-mesh-msgs \
    ros-noetic-dynamic-reconfigure \
    ros-noetic-image-transport \
    ros-noetic-tf2-ros \
    ros-noetic-xacro \
    ros-noetic-robot-state-publisher \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-cv-bridge \
    ros-noetic-image-view \
    python3-opencv \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Build the CPU FMCW radar simulator and its Gazebo plugins from pinned ROS Noetic revisions.
COPY docker/radarays_openmp.patch /tmp/radarays_openmp.patch
RUN mkdir -p /tmp/radarays_ws/src && \
    git clone https://github.com/uos/rmagine.git /tmp/radarays_ws/src/rmagine && \
    git -C /tmp/radarays_ws/src/rmagine checkout 2962242278193dcf8d63697ab3c93900a5513a13 && \
    git clone https://github.com/uos/rmagine_gazebo_plugins.git /tmp/radarays_ws/src/rmagine_gazebo_plugins && \
    git -C /tmp/radarays_ws/src/rmagine_gazebo_plugins checkout ad5aba575b9a576bcc448b3a811c5ee70ea5c9d9 && \
    git clone https://github.com/uos/radarays_ros.git /tmp/radarays_ws/src/radarays_ros && \
    git -C /tmp/radarays_ws/src/radarays_ros checkout e8a466ada8a3c8b84f71f33e2370b2107188a232 && \
    git clone https://github.com/uos/radarays_gazebo_plugins.git /tmp/radarays_ws/src/radarays_gazebo_plugins && \
    git -C /tmp/radarays_ws/src/radarays_gazebo_plugins checkout 6ae1336e72b2f4a1aef201adeb54170b1daca7d4 && \
    patch -d /tmp/radarays_ws/src/radarays_ros -p1 < /tmp/radarays_openmp.patch && \
    sed -i 's/"range_max", double_t, 0, "Maximum range", 600.0/"range_max", double_t, 0, "Maximum range", 45.0/' /tmp/radarays_ws/src/radarays_ros/cfg/RadarModel.cfg && \
    sed -i 's/"n_cells",     int_t,    0, "Number of ranges measurements per beam", 3424/"n_cells",     int_t,    0, "Number of ranges measurements per beam", 1024/' /tmp/radarays_ws/src/radarays_ros/cfg/RadarModel.cfg && \
    sed -i 's/"n_samples",        int_t,    0, "Samples per beam", 10/"n_samples",        int_t,    0, "Samples per beam", 3/' /tmp/radarays_ws/src/radarays_ros/cfg/RadarModel.cfg && \
    sed -i 's/"n_reflections",    int_t,    0, "Maximum Reflections per sample", 4/"n_reflections",    int_t,    0, "Maximum Reflections per sample", 2/' /tmp/radarays_ws/src/radarays_ros/cfg/RadarModel.cfg && \
    sed -i '/ros::NodeHandle gazebo_nh("~");/a\    std::string sensor_name = this->Name(); const auto separator = sensor_name.rfind("::"); if(separator != std::string::npos) sensor_name = sensor_name.substr(separator + 2);' /tmp/radarays_ws/src/radarays_gazebo_plugins/src/radarays_embree_gzplugin.cpp && \
    sed -i 's@std::make_shared<ros::NodeHandle>(gazebo_nh, "radarays")@std::make_shared<ros::NodeHandle>(gazebo_nh, "radarays/" + sensor_name)@' /tmp/radarays_ws/src/radarays_gazebo_plugins/src/radarays_embree_gzplugin.cpp && \
    cmake -S /tmp/radarays_ws/src/rmagine -B /tmp/rmagine_build \
      -DCMAKE_INSTALL_PREFIX=/usr/local -DRMAGINE_BUILD_TOOLS=OFF \
      -DRMAGINE_BUILD_TESTS=OFF -DRMAGINE_BUILD_EXAMPLES=OFF \
      -DRMAGINE_BUILD_DOCS=OFF -DRMAGINE_BUILD_EXPERIMENTS=OFF \
      -DRMAGINE_OUSTER_DISABLE=ON -DRMAGINE_CUDA_DISABLE=ON \
      -DRMAGINE_OPTIX_DISABLE=ON -DRMAGINE_VULKAN_DISABLE=ON \
      -DRMAGINE_VULKAN_CUDA_INTEROP_DISABLE=ON && \
    cmake --build /tmp/rmagine_build -j2 && cmake --install /tmp/rmagine_build && ldconfig && \
    mkdir -p /tmp/radarays_catkin/src && \
    ln -s /tmp/radarays_ws/src/radarays_ros /tmp/radarays_catkin/src/radarays_ros && \
    ln -s /tmp/radarays_ws/src/rmagine_gazebo_plugins /tmp/radarays_catkin/src/rmagine_gazebo_plugins && \
    ln -s /tmp/radarays_ws/src/radarays_gazebo_plugins /tmp/radarays_catkin/src/radarays_gazebo_plugins && \
    cd /tmp/radarays_catkin && . /opt/ros/noetic/setup.sh && \
    catkin_make install -j2 -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic && \
    cp /tmp/radarays_catkin/devel/lib/libradarays_embree_gzplugin.so /opt/ros/noetic/lib/ && \
    cp /tmp/radarays_catkin/devel/lib/libradarays_embree_sensors_gzregister.so /opt/ros/noetic/lib/ && \
    cp /tmp/radarays_catkin/devel/lib/libradarays_embree_ros_gzplugin.so /opt/ros/noetic/lib/ && \
    cp /tmp/radarays_catkin/devel/lib/librmagine_gazebo_helper.so /opt/ros/noetic/lib/ && \
    rm -rf /tmp/radarays_ws /tmp/radarays_catkin /tmp/rmagine_build

WORKDIR /workspace

# The radar plugin creates one simulator per OpenMP thread. Keep CPU usage
# predictable when the two FMCW sensors update at the same time.
ENV OMP_NUM_THREADS=1

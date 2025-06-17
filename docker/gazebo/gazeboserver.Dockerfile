ARG BASE_IMAGE=osrf/ros:humble-desktop-full

################################ Source ################################
FROM ${BASE_IMAGE} AS source

WORKDIR ${AMENT_WS}/src

# Copy in source code 
COPY src/gazebo gazebo

# Scan for rosdeps
RUN apt-get -qq update && rosdep update && \
    rosdep install --from-paths . --ignore-src -r -s \
        | grep 'apt-get install' \
        | awk '{print $3}' \
        | sort  > /tmp/colcon_install_list

################################# Dependencies ################################
FROM ${BASE_IMAGE} AS dependencies

# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys A4B469963BF863CC
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6 -y

RUN apt install -y lsb-release wget gnupg
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get -y update
# The following apt-get install line is commented out due to missing packages in the ROS 2 apt repository.
# RUN apt-get -y install ros-${ROS_DISTRO}-ros-gz ignition-fortress
RUN echo $GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib

# Install Rosdep requirements
COPY --from=source /tmp/colcon_install_list /tmp/colcon_install_list
RUN apt-fast install -qq -y --no-install-recommends $(cat /tmp/colcon_install_list)

# Copy in source code from source stage
WORKDIR ${AMENT_WS}
COPY --from=source ${AMENT_WS}/src src

# Add ROS 2 GPG key and apt source
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Install minimal ROS 2 base and Gazebo simulation packages to avoid 404 errors
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*
# RUN apt-get update && apt-get install -y ros-humble-desktop && rm -rf /var/lib/apt/lists/*  # (Commented out due to 404 errors)

# Attempt to install additional packages, ignore errors if missing
RUN apt-get update && apt-get install -y ros-humble-foxglove-bridge ros-humble-ros-gz-bridge ros-humble-ros-gz-sim ignition-fortress || true

# Install tf2 dependencies explicitly for simulation
RUN apt-get update && apt-get install -y \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    && rm -rf /var/lib/apt/lists/*

# Dependency Cleanup
WORKDIR /
RUN apt-get -qq autoremove -y && apt-get -qq autoclean && apt-get -qq clean && \
    rm -rf /root/* /root/.ros /tmp/* /var/lib/apt/lists/* /usr/share/doc/*

################################ Build ################################
FROM dependencies AS build

# Build ROS2 packages
WORKDIR ${AMENT_WS}
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base ${WATONOMOUS_INSTALL}

# Source and Build Artifact Cleanup 
RUN rm -rf src/* build/* devel/* install/* log/*

# Entrypoint will run before any CMD on launch. Sources ~/opt/<ROS_DISTRO>/setup.bash and ~/ament_ws/install/setup.bash
COPY docker/wato_ros_entrypoint.sh ${AMENT_WS}/wato_ros_entrypoint.sh
ENTRYPOINT ["./wato_ros_entrypoint.sh"]

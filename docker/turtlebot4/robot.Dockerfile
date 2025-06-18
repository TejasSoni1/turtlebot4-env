FROM osrf/ros:humble-desktop

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-turtlebot4-desktop \
        ros-humble-turtlebot4-navigation && \
    rm -rf /var/lib/apt/lists/*

ENV ROS_WS=/root/turtlebot4_ws
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

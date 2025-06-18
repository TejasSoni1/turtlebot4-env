FROM osrf/ros:humble-desktop

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-humble-turtlebot4-simulator && \
    rm -rf /var/lib/apt/lists/*

COPY ros_entrypoint.sh /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

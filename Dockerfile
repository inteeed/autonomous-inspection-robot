FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV TURTLEBOT3_MODEL=waffle_pi

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-opencv \
    python3-pytest \
    ros-humble-cv-bridge \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-turtlebot3-bringup \
    ros-humble-turtlebot3-description \
    ros-humble-turtlebot3-gazebo \
    ros-humble-v4l2-camera \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
COPY ros2_ws/src ./ros2_ws/src

RUN . /opt/ros/humble/setup.sh && \
    cd /workspace/ros2_ws && \
    colcon build --packages-select inspection_robot

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "inspection_robot", "bringup.launch.py"]

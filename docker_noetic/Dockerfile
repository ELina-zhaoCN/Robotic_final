# Ubuntu 20.04 + ROS Noetic for real robot mapping on Ubuntu 22.04 host
FROM ros:noetic-robot

ENV TURTLEBOT3_MODEL=waffle

# Install ROS packages for mapping and navigation
RUN apt-get update && apt-get install -y \
  ros-noetic-move-base \
  ros-noetic-amcl \
  ros-noetic-gmapping \
  ros-noetic-map-server \
  ros-noetic-dwa-local-planner \
  ros-noetic-turtlebot3 \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-turtlebot3-slam \
  ros-noetic-turtlebot3-bringup \
  ros-noetic-turtlebot3-description \
  ros-noetic-xacro \
  ros-noetic-teleop-twist-keyboard \
  ros-noetic-dynamic-reconfigure \
  ros-noetic-message-generation \
  ros-noetic-actionlib \
  ros-noetic-tf2-geometry-msgs \
  && rm -rf /var/lib/apt/lists/*

# Workspace will be mounted at runtime, create dir only
RUN mkdir -p /catkin_ws/src

WORKDIR /catkin_ws

# Source ROS env by default (use roscore, roslaunch directly in container)
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

CMD ["/bin/bash"]

# Ubuntu 20.04 + ROS Noetic，用于在 Ubuntu 22.04 主机上跑真机建图
FROM ros:noetic-robot

ENV TURTLEBOT3_MODEL=waffle

# 安装建图和导航所需全部 ROS 包
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

# 工作空间会在运行时挂载，这里只建目录
RUN mkdir -p /catkin_ws/src

WORKDIR /catkin_ws

# 默认先 source 环境（进入容器后可直接用 roscore、roslaunch）
RUN echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

CMD ["/bin/bash"]

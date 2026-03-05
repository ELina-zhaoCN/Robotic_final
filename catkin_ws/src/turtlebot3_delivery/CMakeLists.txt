cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_delivery)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

# ── Custom message generation ──────────────────────────────────────────────────
add_message_files(
  FILES
  RobotState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

# ── catkin package declaration ─────────────────────────────────────────────────
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    actionlib
    actionlib_msgs
    move_base_msgs
    tf
    tf2_ros
    tf2_geometry_msgs
    message_runtime
)

# ── Install Python scripts ─────────────────────────────────────────────────────
catkin_install_python(PROGRAMS
  scripts/fsm_core.py
  scripts/obstacle_detector.py
  scripts/navigation_control.py
  scripts/task_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ── Install launch and config files ───────────────────────────────────────────
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg
)

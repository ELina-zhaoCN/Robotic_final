# Custom FSM state message for TurtleBot3 Waffle food delivery robot
# Published on /robot_state topic at 10Hz

# State ID constants
uint8 IDLE              = 0
uint8 NAVIGATING        = 1
uint8 DELIVERING_FOOD   = 2
uint8 AVOIDING_OBSTACLE = 3
uint8 TASK_COMPLETED    = 4

# Current FSM state
uint8  state_id
string state_name

# Transition event that caused this state (empty string if no recent transition)
string trigger_event

# Sensor data snapshot at time of publish
float32 min_obstacle_dist     # Minimum LiDAR distance in front sector (meters)
bool    obstacle_detected     # True if any reading < 0.5m in front ±180°

# Current navigation goal (map frame)
float64 goal_x
float64 goal_y
float64 goal_yaw

# Time robot has been in current state (seconds)
float32 state_duration

# ROS header for timestamp
Header header

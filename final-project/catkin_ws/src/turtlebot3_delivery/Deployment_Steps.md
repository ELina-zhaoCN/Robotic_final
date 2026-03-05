# Deployment Steps — TurtleBot3 Waffle Food Delivery FSM

Complete guide from first boot to one-click execution.
Tested on: Ubuntu 20.04 + ROS Noetic + TurtleBot3 Waffle

---

## Prerequisites

| Item | Requirement |
|------|------------|
| OS   | Ubuntu 20.04 LTS |
| ROS  | Noetic Ninjemys (full desktop install) |
| Robot | TurtleBot3 Waffle (OpenCR + Raspberry Pi 4) |
| LiDAR | SLAMTEC RPLIDAR A1M8 (USB `/dev/ttyUSB0`) |
| Camera | Raspberry Pi Camera Module v2 (RGB) |
| PC ↔ Robot | Same Wi-Fi network |

---

## Step 1 — ROS Environment Setup (PC + Robot)

### 1.1 PC Setup

```bash
# Install ROS Noetic (if not already installed)
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# Install navigation dependencies
sudo apt install -y \
  ros-noetic-move-base \
  ros-noetic-amcl \
  ros-noetic-gmapping \
  ros-noetic-map-server \
  ros-noetic-dwa-local-planner \
  ros-noetic-turtlebot3 \
  ros-noetic-turtlebot3-msgs \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-turtlebot3-slam \
  ros-noetic-dynamic-reconfigure

# Add to ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### 1.2 Robot (Raspberry Pi) Setup

```bash
# SSH into robot
ssh ubuntu@<ROBOT_IP>

# Install TurtleBot3 packages
sudo apt install -y ros-noetic-turtlebot3 ros-noetic-turtlebot3-bringup

# Set ROS_MASTER_URI to PC
echo "export ROS_MASTER_URI=http://<PC_IP>:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=<ROBOT_IP>" >> ~/.bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

### 1.3 PC Network Config

```bash
# On PC — add to ~/.bashrc
echo "export ROS_MASTER_URI=http://<PC_IP>:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=<PC_IP>" >> ~/.bashrc
source ~/.bashrc

# Start ROS master on PC
roscore &
```

---

## Step 2 — Build the Package

```bash
# Create catkin workspace (skip if already exists)
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws

# Copy the turtlebot3_delivery package
cp -r /path/to/turtlebot3_delivery ~/catkin_ws/src/

# Build
cd ~/catkin_ws
catkin_make

# Source workspace
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/catkin_ws/devel/setup.bash

# Make scripts executable
chmod +x ~/catkin_ws/src/turtlebot3_delivery/scripts/*.py
```

**Verify build:**

```bash
rospack find turtlebot3_delivery
# Expected output: /home/<user>/catkin_ws/src/turtlebot3_delivery
```

---

## Step 3 — Hardware Bringup (On Robot)

```bash
# Terminal 1 (SSH into robot)
ssh ubuntu@<ROBOT_IP>
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

You should see: `Odom frame is: odom` and `/scan` data flowing.

**Verify RPLIDAR A1M8:**

```bash
# On PC — check scan data
rostopic hz /scan
# Expected: ~5–10 Hz
rostopic echo /scan --noarr | head -5
# Expected: ranges: [0.xx, 0.xx, ...] with 360 values
```

---

## Step 4 — Build the Restaurant Map (SLAM)

```bash
# Terminal 2 (PC) — Launch mapping
roslaunch turtlebot3_delivery mapping.launch

# Terminal 3 (PC) — Drive robot to map the restaurant
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Drive the robot slowly around ALL areas of the restaurant.
Watch RViz — ensure all walls and tables appear on the map.

**Save the map:**

```bash
# Terminal 4 (PC) — Save when mapping is complete
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/restaurant_map
```

This creates:
- `~/maps/restaurant_map.yaml`
- `~/maps/restaurant_map.pgm`

**Kill all mapping terminals** before proceeding.

---

## Step 5 — Configure Delivery Locations

Edit the location table in `config/fsm_params.yaml`:

```yaml
fsm:
  locations:
    kitchen:  [0.0,   0.0,   0.0]   # Starting position (must match AMCL init pose)
    table_1:  [3.0,   2.0,   0.0]   # Adjust to actual table coordinates
    table_2:  [3.0,  -2.0,   0.0]
    table_3:  [6.0,   0.0,  90.0]
    # Add more tables as needed
```

**How to find table coordinates:**
1. Run `navigation.launch` (Step 6)
2. In RViz, click **Publish Point** tool, click on the table location
3. Note the (x, y) from the terminal output

---

## Step 6 — One-Click Delivery System Launch

```bash
# Terminal 1 (Robot) — Hardware bringup (if not already running)
ssh ubuntu@<ROBOT_IP>
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# Terminal 2 (PC) — Launch complete delivery system
roslaunch turtlebot3_delivery delivery_fsm.launch \
  map_file:=$HOME/maps/restaurant_map.yaml

# Wait for: "[FSM] move_base connected."
# Wait for: "[FSM] Entering IDLE state."
```

**Set initial pose in RViz:**
1. Click **2D Pose Estimate** button
2. Click on the kitchen location on the map
3. Drag to set orientation (facing forward)
4. Wait for AMCL particles to converge (green arrows cluster)

---

## Step 7 — Send Delivery Tasks

```bash
# By location name (recommended):
rostopic pub /delivery_task std_msgs/String "data: 'table_1'"

# By raw coordinates (x, y):
rostopic pub /delivery_task std_msgs/String "data: '3.5,2.0'"

# By coordinates with yaw (x, y, degrees):
rostopic pub /delivery_task std_msgs/String "data: '3.5,2.0,90'"
```

**Monitor task execution:**

```bash
# Watch FSM state changes in real-time
rostopic echo /robot_state

# Watch obstacle detection
rostopic echo /obstacle_detected
rostopic echo /min_obstacle_dist

# Watch task completion
rostopic echo /finish_task
```

---

## Step 8 — Verify State Transitions

Expected terminal output for a successful delivery:

```
[FSM] TRANSITION: IDLE → NAVIGATING  [trigger: task_received]
[FSM] Nav goal sent: (3.00, 2.00) speed=0.15m/s
[FSM] TRANSITION: NAVIGATING → DELIVERING_FOOD  [trigger: near_goal]
... (robot delivers normally) ...
[FSM] TRANSITION: DELIVERING_FOOD → TASK_COMPLETED  [trigger: goal_reached]
[FSM] Task complete signal published to /finish_task.
[FSM] TRANSITION: TASK_COMPLETED → IDLE  [trigger: task_confirmed]
```

Expected output with obstacle:

```
[FSM] OBSTACLE at 0.342m (< 0.50m threshold). Triggering AVOIDANCE.
[FSM] Avoidance direction: LEFT (left=0.82m, right=0.31m)
[FSM] TRANSITION: DELIVERING_FOOD → AVOIDING_OBSTACLE  [trigger: obstacle_detected]
... (robot detours around obstacle) ...
[FSM] Path clear for 2.0s (>= 2.0s). Resuming delivery.
[FSM] TRANSITION: AVOIDING_OBSTACLE → DELIVERING_FOOD  [trigger: path_clear_2s]
[FSM] Delivery speed restored: 0.15m/s
```

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| `[FSM] move_base not available!` | Run `navigation.launch` first |
| `/scan` empty or no data | Check RPLIDAR USB: `ls /dev/ttyUSB*`; re-plug if missing |
| Robot doesn't move | Verify `ROS_MASTER_URI` matches on PC and robot |
| AMCL pose jumping | Re-set 2D Pose Estimate in RViz more precisely |
| Obstacle avoidance loops | Increase `clear_duration` in `fsm_params.yaml` to 3.0s |
| Navigation timeout | Reduce `nav_goal_timeout` or clear map obstructions |
| `catkin_make` fails on RobotState | Run `catkin_make --cmake-args -DCATKIN_WHITELIST_PACKAGES="turtlebot3_delivery"` |

---

## Rosbag Replay

Session bags are auto-saved to `/tmp/delivery_session_*.bag`.

```bash
# Replay a session
rosbag play /tmp/delivery_session_<timestamp>.bag

# Inspect recorded topics
rosbag info /tmp/delivery_session_<timestamp>.bag
```

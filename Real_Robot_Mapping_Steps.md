# Real Robot Mapping — Complete Steps

When using a **real TurtleBot**, you must **connect and start the robot first**, then run mapping on the PC. Order matters.

---

## Prerequisites

- PC and robot on the **same WiFi** (or same LAN).
- PC: Ubuntu 20.04 + **ROS Noetic** (real robot mapping uses ROS 1, not ROS 2).
- Robot: TurtleBot3 Waffle (Raspberry Pi + OpenCR + RPLIDAR A1).
- Know the robot IP (e.g. `192.168.1.100`) and PC IP (e.g. `192.168.1.50`).

---

## Part 1: Connect the Robot (do this before each mapping session)

### 1. On PC: Set up ROS and start roscore

Open **Terminal 1** and run:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle

# Use this PC as ROS master (replace <PC_IP> with your PC's IP)
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<PC_IP>

# Start ROS master (keep this terminal open)
roscore
```

**Important**: `ROS_HOSTNAME` must be **your PC's IP**, not the robot's. Example:

```bash
export ROS_MASTER_URI=http://10.18.123.45:11311
export ROS_HOSTNAME=10.18.123.45
roscore
```

If you see **`roscore` or `roslaunch` not found**: your system has ROS 2, not ROS 1. Real TurtleBot3 mapping needs **ROS 1 Noetic**. See "No ROS 1 on this machine" at the end.

---

### 2. On robot: SSH in and start chassis + LiDAR

Open **Terminal 2**, SSH to the robot (replace `<ROBOT_IP>` with the robot's IP):

```bash
ssh ubuntu@<ROBOT_IP>
```

After login, on the **robot** run:

```bash
source /opt/ros/noetic/setup.bash
export TURTLEBOT3_MODEL=waffle

# Point robot to your PC's roscore
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<ROBOT_IP>

# Start robot bringup: chassis + LiDAR (keep running)
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

If you see `Odom frame is: odom` with no errors, the robot is connected and publishing.

---

### 3. On PC: Verify robot data

Open **Terminal 3** (on PC, not SSH):

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<PC_IP>

# Check LiDAR rate (~5–10 Hz)
rostopic hz /scan
```

If you see a frequency, **connection is OK**. Press `Ctrl+C` to stop.

---

## Part 2: Run SLAM Mapping on PC

Keep roscore and `turtlebot3_robot.launch` running.

### 4. Start mapping on PC ("open SLAM")

Open **Terminal 4** on the PC:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<PC_IP>

roslaunch turtlebot3_delivery mapping.launch
```

- **RViz** opens and gmapping starts ("SLAM on").
- No localization needed during mapping.
- Proceed when the map appears in RViz.

---

### 5. Teleop the robot to scan

Open **Terminal 5** on the PC:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<PC_IP>

rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

- Keys are shown in the terminal (e.g. i/j/k/l).
- **Click the terminal** first, then drive the robot through the area (kitchen, aisles, around tables).
- Watch the map build in RViz, then save.

---

### 6. Save the map (on PC)

Open **Terminal 6** on the PC:

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/maps
export ROS_MASTER_URI=http://<PC_IP>:11311
export ROS_HOSTNAME=<PC_IP>

rosrun map_server map_saver -f ~/maps/restaurant_map
```

This creates in `~/maps/`:

- `restaurant_map.yaml`
- `restaurant_map.pgm`

Then stop mapping and teleop with `Ctrl+C`. For navigation later:

```bash
roslaunch turtlebot3_delivery delivery_fsm.launch map_file:=$HOME/maps/restaurant_map.yaml
```

Use **2D Pose Estimate** in RViz to set the robot's initial pose in the kitchen.

---

## Summary

| Step | Where | Action |
|------|-------|--------|
| 1 | PC Terminal 1 | Set `ROS_MASTER_URI`, `ROS_HOSTNAME`, run `roscore` |
| 2 | SSH to robot | Set same `ROS_MASTER_URI`, run `turtlebot3_robot.launch` |
| 3 | PC | `rostopic hz /scan` to verify data |
| 4 | PC | `roslaunch turtlebot3_delivery mapping.launch` |
| 5 | PC | `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` |
| 6 | PC | `rosrun map_server map_saver -f ~/maps/restaurant_map` |

**Real robot mapping requires connecting and starting the robot before running mapping on the PC.**

For more details: `catkin_ws/src/turtlebot3_delivery/Deployment_Steps.md`.

---

## No ROS 1 on this machine?

Your PC has **ROS 2 Humble** but not **ROS 1 Noetic**, so `roscore` and `roslaunch` are missing. Real TurtleBot3 mapping needs ROS 1.

### If system is Ubuntu 20.04

Install ROS Noetic (can coexist with ROS 2):

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full
sudo apt install -y ros-noetic-move-base ros-noetic-amcl ros-noetic-gmapping \
  ros-noetic-map-server ros-noetic-dwa-local-planner ros-noetic-turtlebot3 \
  ros-noetic-turtlebot3-msgs ros-noetic-turtlebot3-navigation \
  ros-noetic-turtlebot3-slam ros-noetic-teleop-twist-keyboard
```

Then before each ROS 1 session:

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle
```

### If system is Ubuntu 22.04 or newer

Noetic officially supports Ubuntu 20.04 only. Options:

- **Option A**: Use another PC with **Ubuntu 20.04** for mapping/navigation, same WiFi as robot.
- **Option B**: Use **VM or Docker** with Ubuntu 20.04, install Noetic and `catkin_ws`, use bridged networking so VM and robot are on the same subnet.

After Noetic is installed, follow Part 1 and Part 2 above.

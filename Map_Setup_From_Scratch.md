# Map Setup From Scratch

This guide explains how to build a map, save it, and configure delivery goal points when you have no existing map. The project supports two environments; choose the one you use.

---

## Option 1: ROS 2 + TurtleBot4 Ignition Simulation

(For ROS 2 Humble and the simulation in README.)

### 1. Build map (SLAM on)

With no map yet, use **SLAM mode** to map while driving:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  nav2:=true slam:=true localization:=false rviz:=true world:=warehouse
```

- `slam:=true`: map while driving
- `localization:=false`: no localization during mapping

### 2. Drive in simulation to scan

- Use keyboard/gamepad in RViz or simulation to drive through kitchen, aisles, around tables.
- Follow walls and corners so all obstacles appear on the map.
- Keep simulation running, then save the map.

### 3. Save the map

In a new terminal:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/restoran
```

This creates:

- `~/maps/restoran.yaml`
- `~/maps/restoran.pgm`

Change the path after `-f` if you prefer a different location.

### 4. Run with saved map (SLAM off)

After mapping, use the saved map + localization:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  nav2:=true slam:=false localization:=true rviz:=true world:=warehouse \
  map:=/home/YOUR_USERNAME/maps/restoran.yaml
```

Replace with your actual path to `restoran.yaml`.

### 5. Set initial pose in RViz

- Click **2D Pose Estimate**, click on the map where the robot is, drag for heading.
- After particles converge, use Nav2 or `gui.py` for delivery.

---

## Option 2: ROS 1 Noetic + TurtleBot3 (real robot or Gazebo)

(For the `turtlebot3_delivery` package in `catkin_ws`.)

### 1. Environment and dependencies

- Ubuntu 20.04 + ROS Noetic
- Installed: `ros-noetic-gmapping`, `ros-noetic-map-server`, `ros-noetic-turtlebot3-slam`, `ros-noetic-teleop-twist-keyboard`
- Real robot: SSH to robot, run `roslaunch turtlebot3_bringup turtlebot3_robot.launch` so `/scan` and `/odom` are published.

### 2. Start mapping

With the robot connected (or in simulation):

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export TURTLEBOT3_MODEL=waffle

roslaunch turtlebot3_delivery mapping.launch
```

### 3. Teleop to scan

In a new terminal:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

- Drive through the whole restaurant: kitchen, aisles, around tables.
- In RViz, confirm walls, tables, and passages appear on `/map`.

### 4. Save the map

After scanning:

```bash
mkdir -p ~/maps
rosrun map_server map_saver -f ~/maps/restaurant_map
```

This creates:

- `~/maps/restaurant_map.yaml`
- `~/maps/restaurant_map.pgm`

Then stop mapping and teleop.

### 5. Run navigation with saved map

For delivery/navigation, specify the map:

```bash
roslaunch turtlebot3_delivery delivery_fsm.launch map_file:=$HOME/maps/restaurant_map.yaml
```

Or navigation only:

```bash
roslaunch turtlebot3_delivery navigation.launch map_file:=$HOME/maps/restaurant_map.yaml
```

Use **2D Pose Estimate** in RViz to set the robot's initial pose in the kitchen.

### 6. Configure delivery goal points (tables, kitchen)

After mapping, add **kitchen** and **table** coordinates so the robot can navigate by name.

**Method A: Click in RViz**

1. Start `navigation.launch` or `delivery_fsm.launch` and set initial pose.
2. In RViz, use **Publish Point** and click: kitchen, table_1, table_2, etc.
3. Note the printed `(x, y)` in the terminal.
4. Add them to the config file.

**Method B: Edit config directly**

Edit at least one of:

- `catkin_ws/src/turtlebot3_delivery/config/fsm_map_config.yaml`
- `catkin_ws/src/turtlebot3_delivery/config/fsm_params.yaml`

Under `locations:`:

```yaml
locations:
  kitchen:  [0.0,   0.0,   0.0]   # Kitchen start
  table_1:  [3.0,   2.0,   0.0]   # Table 1: x, y, heading (deg)
  table_2:  [3.0,  -2.0,   0.0]
  table_3:  [6.0,   0.0,  90.0]
```

- First two numbers: x, y in map frame (meters).
- Third: heading when arrived (0 = +x, 90 = +y).
- Place goals in free space, ~0.5 m from walls/tables.

You can also use `mark_goal_points.py` to click in RViz and auto-fill `fsm_map_config.yaml`.

---

## Summary

| Step | ROS 2 (TurtleBot4 Ignition) | ROS 1 (TurtleBot3) |
|------|-----------------------------|-------------------|
| 1. Map | `slam:=true`, drive to scan | `mapping.launch` + `teleop_twist_keyboard` |
| 2. Save | `nav2_map_server map_saver_cli -f ~/maps/restoran` | `map_saver -f ~/maps/restaurant_map` |
| 3. Run | `slam:=false`, `map:=/path/to/restoran.yaml` | `delivery_fsm.launch map_file:=...` |
| 4. Goals | Use coordinates/names in GUI | Edit `fsm_map_config.yaml` / `fsm_params.yaml` `locations` |

Choose the option that matches your setup (ROS 2 simulation or ROS 1 real robot/simulation).

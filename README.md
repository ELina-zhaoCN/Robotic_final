# TurtleBot3 Restaurant Delivery Robot

TurtleBot3 Waffle restaurant delivery system. Supports ROS 1 Noetic (real robot) and ROS 2 Humble (simulation).

---

## File Reference — What Each File Does

### Map (Static Environment Data)

| File / Dir | Purpose |
|------------|---------|
| `maps/restaurant_map.yaml` | Map metadata: resolution, origin, image path |
| `maps/restaurant_map.pgm` | Occupancy grid image (walls, free space) |
| `catkin_ws/.../config/fsm_map_config.yaml` | Goal coordinates (kitchen, table_1–5), zones, waypoints |
| `catkin_ws/.../config/gmapping_params.yaml` | SLAM parameters for building the map |
| `Map_Info.md` | Map documentation (size, resolution, usage) |

The **map** is the pre-built static representation of the restaurant layout. It is created once (via SLAM), saved, then loaded by `map_server` for localization and path planning.

---

### Fusion Algorithm (FSM + Sensor Fusion)

| File | Purpose |
|------|---------|
| `scripts/fsm_map_fusion.py` | **Master controller**: fuses map knowledge + LiDAR + AMCL into a 5-state FSM (IDLE → NAVIGATING → DELIVERING ⇄ AVOIDING → TASK_COMPLETED). Sends goals to move_base, overrides with avoidance twist when obstacles detected. |
| `scripts/obstacle_detector.py` | Parses `/scan` at 100 Hz, 0.5 m threshold, sector analysis (front/left/right). Publishes `obstacle_detected`, `min_obstacle_dist`. |
| `scripts/task_manager.py` | Receives delivery tasks, resolves location names to map coordinates, publishes goals. |
| `scripts/navigation_control.py` | Sends MoveBase goals, monitors status, publishes `cmd_vel`. |
| `scripts/fsm_core.py` | Base FSM engine (used by delivery stack). |
| `config/fsm_params.yaml` | FSM parameters: obstacle threshold, speeds, timeouts, topics. |

*(Scripts and config under `catkin_ws/src/turtlebot3_delivery/`.)*

The **fusion algorithm** combines the static map with real-time LiDAR and AMCL pose to decide when to navigate, when to avoid obstacles, and when the task is complete.

---

### Map vs Fusion — Summary

| Concept | Role |
|--------|------|
| **Map** | Static occupancy grid + goal coordinates. Built once, used for localization (AMCL) and global path planning. |
| **Fusion algorithm** | Runtime logic: reads map goals + LiDAR + AMCL pose → drives FSM states, sends navigation/avoidance commands. |

---

### Other Project Files

| File / Dir | Purpose |
|------------|---------|
| `gui.py` | ROS 2 delivery GUI (CustomTkinter, Nav2). Sends waypoints, shows robot position. |
| `requirements.txt` | Python deps (customtkinter) for gui.py |
| `docker_noetic/` | Docker env for Ubuntu 20.04 + ROS Noetic (real robot mapping on Ubuntu 22.04 host) |
| `launch/delivery_fsm.launch` | One-click: navigation + obstacle_detector + task_manager + fsm_core |
| `launch/navigation.launch` | AMCL + map_server + move_base |
| `launch/mapping.launch` | gmapping SLAM + RViz (full stack) |
| `launch/mapping_pc_only.launch` | gmapping + RViz on PC only (robot runs bringup elsewhere) |
| `config/costmap_params.yaml` | Global/local costmap config |
| `config/dwa_params.yaml` | DWA local planner speeds |
| `config/move_base_params.yaml` | Planner frequencies, recovery behaviors |
| `scripts/mark_goal_points.py` | Click in RViz to mark kitchen/table coords → writes fsm_map_config.yaml |
| `scripts/map_tools.py` | Save/load map to ~/maps/ |
| `scripts/keyboard_mapping.py` | Teleop during SLAM |
| `Map_Setup_From_Scratch.md` | How to build map and set goal points |
| `Real_Robot_Mapping_Steps.md` | Real robot mapping workflow |
| `PRD.md` | Product requirements document |
| `catkin_ws/src/turtlebot3_delivery/Map_Architecture.md` | System architecture diagram |
| `catkin_ws/src/turtlebot3_delivery/Obstacle_Avoidance_Algorithm.md` | 0.5 m threshold, sector design |
| `catkin_ws/src/turtlebot3_delivery/State_Transition_Diagram.md` | FSM state diagram |
| `catkin_ws/src/turtlebot3_delivery/Deployment_Steps.md` | Deployment and troubleshooting |

*Launch/config/scripts paths above are under `catkin_ws/src/turtlebot3_delivery/` unless noted.*

---

## Project Structure

```
├── maps/                    # Map files (restaurant_map)
├── catkin_ws/src/turtlebot3_delivery/
├── docker_noetic/           # Docker mapping env
├── gui.py                   # ROS 2 delivery GUI (Nav2)
├── requirements.txt
├── Map_Info.md
├── Real_Robot_Mapping_Steps.md
└── Map_Setup_From_Scratch.md
```

## Quick Start

### ROS 1 Real Robot Delivery (default: project maps/)

```bash
cd catkin_ws && catkin_make && source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_delivery delivery_fsm.launch
```

### ROS 2 Simulation + GUI

```bash
pip install -r requirements.txt
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
  nav2:=true slam:=false localization:=true rviz:=true \
  map:=$(pwd)/maps/restaurant_map.yaml
# In another terminal
python3 gui.py
```

## Dependencies

- **ROS 1**: Ubuntu 20.04 + ROS Noetic (real robot mapping/navigation)
- **ROS 2**: Ubuntu 22.04 + ROS 2 Humble (simulation, GUI)
- **Python**: `pip install -r requirements.txt`

## Documentation

| Document | Description |
|----------|-------------|
| [Real_Robot_Mapping_Steps.md](Real_Robot_Mapping_Steps.md) | Real robot mapping workflow |
| [Map_Setup_From_Scratch.md](Map_Setup_From_Scratch.md) | Map building and goal point setup |
| [Map_Info.md](Map_Info.md) | Current map parameters |
| [docker_noetic/README.md](docker_noetic/README.md) | Docker mapping environment |
| [Map_Architecture.md](catkin_ws/src/turtlebot3_delivery/Map_Architecture.md) | System architecture (map vs fusion) |
| [Obstacle_Avoidance_Algorithm.md](catkin_ws/src/turtlebot3_delivery/Obstacle_Avoidance_Algorithm.md) | 0.5 m threshold, sector design |

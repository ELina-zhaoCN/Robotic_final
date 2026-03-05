# How to Run

## 1. Launch the Simulation
Open a terminal and start the Gazebo environment with navigation, localization, and RViz enabled.
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py nav2:=true slam:=false localization:=true rviz:=true world:=warehouse map:=/home/ubuntu/turtlebot4_ws/maps/restoran.yaml
```

## 2. Set Initial Pose
In the RViz window that opens, use the **2D Pose Estimate** tool to set the robot's initial position on the map.

## 3. Run the Dashboard
Open a new terminal and launch your control panel.
```bash
python3 dashboard.py
```

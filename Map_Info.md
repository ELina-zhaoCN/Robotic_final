# Current Map Information

## Map Files (included in project)

| Item | Value |
|------|-------|
| **Path** | `maps/restaurant_map.yaml` (in project) |
| **Image** | `maps/restaurant_map.pgm` |
| **Resolution** | 0.05 m/pixel |
| **Size** | 195 × 179 pixels |
| **Physical extent** | ~9.75 m × 8.95 m |
| **Origin** | [-3.14, -5.96, 0] (meters) |

## Usage

### ROS 1 Real Robot Navigation (default: project maps/)

```bash
roslaunch turtlebot3_delivery delivery_fsm.launch
# Or specify: map_file:=$(pwd)/maps/restaurant_map.yaml
```

### ROS 1 Navigation Only (with AMCL)

```bash
roslaunch turtlebot3_delivery navigation.launch
```

### Marking Delivery Goal Points

1. Start `navigation.launch` (with map)
2. In RViz, use **Publish Point** or run `mark_goal_points.py` to mark kitchen and table positions
3. Coordinates are written to `fsm_map_config.yaml` under `locations`

## Related Config Files

- `catkin_ws/src/turtlebot3_delivery/config/fsm_map_config.yaml` — Map path, goal points, zones
- `catkin_ws/src/turtlebot3_delivery/config/fsm_params.yaml` — FSM params, locations backup

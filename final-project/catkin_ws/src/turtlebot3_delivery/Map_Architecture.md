# Map Architecture Design
## TurtleBot3 Waffle — Simulated Restaurant Food Delivery

---

## 1. Overall System Architecture Flowchart

```mermaid
flowchart TD
    subgraph HARDWARE["Hardware Layer"]
        H1["RPLIDAR A1M8\n/scan topic\n5–10Hz, 360°"]
        H2["RGB Camera\n/camera/image_raw\nSLAM texture source"]
        H3["OpenCR + Motors\n/cmd_vel → wheel motion"]
        H4["Odometry\n/odom topic"]
    end

    subgraph MAP_MODULE["① Mapping Module (gmapping)"]
        M1["keyboard_mapping.py\nTeleop driver during SLAM"]
        M2["gmapping node\nSLAM algorithm\nBuilds occupancy grid"]
        M3["map_tools.py\nSaves .yaml + .pgm\nLoads map on boot"]
        M4["mark_goal_points.py\nAnnotates: kitchen\ntable_1~5, aisles"]
        M5["fsm_map_config.yaml\nGoal coordinate database\nZone definitions"]
    end

    subgraph NAV_MODULE["② Navigation Module (move_base)"]
        N1["AMCL\nParticle filter\nLocalization via /scan"]
        N2["Global Costmap\nNavFn planner\nInflation 0.5m"]
        N3["Local Costmap\nDWA planner\n0.5m obstacle buffer"]
        N4["move_base ActionServer\nPath execution\n/move_base/goal"]
    end

    subgraph OBS_MODULE["③ Obstacle Avoidance Module"]
        O1["obstacle_detector.py\nParse /scan every 10ms\nThreshold: 0.5m"]
        O2["Sector Analysis\nfront ±30°\nleft 30–90°\nright -30°–-90°"]
        O3["Avoidance Twist\nlinear=0.05m/s\nangular=0.40rad/s"]
    end

    subgraph FSM_MODULE["④ FSM State Machine Module"]
        F1["fsm_map_fusion.py\nCentral controller\n10Hz loop"]
        F2["IDLE\nKitchen base"]
        F3["NAVIGATING\nGlobal path"]
        F4["DELIVERING_FOOD\nFine tracking"]
        F5["AVOIDING_OBSTACLE\nDetour maneuver"]
        F6["TASK_COMPLETED\nStop + signal"]
    end

    H1 -->|LaserScan| M2
    H1 -->|LaserScan| O1
    H1 -->|LaserScan| N1
    H4 -->|Odometry| M2
    H4 -->|Odometry| N1
    H2 -->|Map texture| M2

    M2 -->|/map OccupancyGrid| M3
    M3 -->|Saved map .yaml| N2
    M4 -->|Goal coords| M5
    M5 -->|Locations| F1
    M1 -->|/cmd_vel| H3

    N1 -->|/amcl_pose| F1
    N2 -->|Global plan| N4
    N3 -->|Local cmd| N4
    N4 -->|/cmd_vel| H3

    O1 --> O2
    O2 -->|obstacle_detected\nmin_dist\nsector_dists| F1
    O3 -->|/cmd_vel| H3

    F1 --> F2
    F2 -->|task_received| F3
    F3 -->|near_goal| F4
    F4 -->|dist<0.5m| F5
    F5 -->|clear 2s| F4
    F4 -->|arrived| F6
    F6 -->|confirmed| F2

    F1 -->|send goal| N4
    F5 --> O3
```

---

## 2. Module Division

### Module ① — Mapping Module
| File | Function |
|------|----------|
| `keyboard_mapping.py` | Drive robot via WASD keys while gmapping builds the map |
| `gmapping_params.yaml` | RPLIDAR A1M8-tuned SLAM parameters |
| `map_tools.py` | Auto-save map to `~/maps/`, load on startup |
| `mark_goal_points.py` | Click-to-mark kitchen & table coordinates in RViz |
| `fsm_map_config.yaml` | Persisted restaurant location database |

### Module ② — Navigation Module
| File | Function |
|------|----------|
| `costmap_common_params.yaml` | Shared costmap settings: inflation_radius=**0.5m** |
| `move_base_params.yaml` | Planner frequencies, recovery behaviors |
| `navigation.launch` | Launches AMCL + map_server + move_base |

### Module ③ — Obstacle Avoidance Module
| File | Function |
|------|----------|
| `obstacle_detector.py` | 100Hz scan parsing, 0.5m threshold, sector analysis |
| `dwa_params.yaml` | Local planner speed caps (delivery=0.15m/s, avoidance=0.05m/s) |

### Module ④ — FSM State Machine Module
| File | Function |
|------|----------|
| `fsm_map_fusion.py` | **Master controller**: fuses map knowledge + LiDAR + AMCL into 5-state FSM |
| `fsm_core.py` | Original FSM engine (used by fsm_map_fusion as base class) |
| `task_manager.py` | Task queue management |

---

## 3. Inter-Module Data Flow

```mermaid
sequenceDiagram
    participant LIDAR as RPLIDAR A1M8
    participant GMAPPING as gmapping
    participant AMCL as AMCL
    participant DETECTOR as obstacle_detector
    participant FSM as fsm_map_fusion
    participant MOVEBASE as move_base
    participant ROBOT as /cmd_vel

    Note over LIDAR,ROBOT: Phase 1 — Mapping (offline, one-time)
    LIDAR->>GMAPPING: /scan
    GMAPPING->>GMAPPING: Build occupancy grid
    GMAPPING-->>ROBOT: map saved to ~/maps/

    Note over LIDAR,ROBOT: Phase 2 — Delivery Operation (runtime)
    LIDAR->>AMCL: /scan (localization)
    AMCL->>FSM: /amcl_pose (current position)
    LIDAR->>DETECTOR: /scan (obstacle check 100Hz)
    DETECTOR->>FSM: /obstacle_detected + /min_obstacle_dist
    FSM->>MOVEBASE: MoveBaseGoal (target table)
    MOVEBASE->>ROBOT: /cmd_vel (navigation)
    FSM->>ROBOT: /cmd_vel (avoidance Twist override)
    FSM->>FSM: /finish_task=True (arrival)
```

### Topic Communication Summary

| Topic | Publisher | Subscriber | Message Type | Rate |
|-------|-----------|------------|-------------|------|
| `/scan` | RPLIDAR driver | gmapping, AMCL, obstacle_detector, fsm | LaserScan | 10Hz |
| `/odom` | OpenCR | gmapping, AMCL, move_base | Odometry | 30Hz |
| `/map` | gmapping / map_server | move_base, RViz | OccupancyGrid | 1Hz |
| `/amcl_pose` | AMCL | fsm_map_fusion | PoseWithCovarianceStamped | 10Hz |
| `/cmd_vel` | move_base / fsm (avoidance) | OpenCR | Twist | 10Hz |
| `/obstacle_detected` | obstacle_detector | fsm_map_fusion | Bool | 100Hz |
| `/min_obstacle_dist` | obstacle_detector | fsm_map_fusion | Float32 | 100Hz |
| `/robot_state` | fsm_map_fusion | task_manager, RViz | String | 10Hz |
| `/finish_task` | fsm_map_fusion | task_manager | Bool | on-event |
| `/delivery_task` | CLI / task_manager | fsm_map_fusion | String | on-event |
| `/marked_goals` | mark_goal_points | fsm_map_fusion | String (JSON) | on-event |

---

## 4. Restaurant Map Zone Layout

```
┌─────────────────────────────────────────────────────────────┐
│                    RESTAURANT MAP                           │
│                                                             │
│  ┌──────────┐    DELIVERY AISLE         ┌───────────────┐  │
│  │          │   ════════════════════   │  DINING AREA  │  │
│  │ KITCHEN  │──►  table_1 (3.0, 2.0)   │               │  │
│  │          │──►  table_2 (3.0,-2.0)   │  T1  T2  T3  │  │
│  │[0.0,0.0] │──►  table_3 (6.0, 0.0)   │               │  │
│  │          │──►  table_4 (6.0, 3.0)   │  T4  T5      │  │
│  └──────────┘──►  table_5 (8.0,-1.5)   └───────────────┘  │
│                                                             │
│  ← Floor tape marks zone boundaries                        │
│  ← Update coordinates in fsm_map_config.yaml after mapping │
└─────────────────────────────────────────────────────────────┘
```

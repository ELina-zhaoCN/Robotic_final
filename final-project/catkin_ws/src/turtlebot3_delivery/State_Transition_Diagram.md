# FSM State Transition Diagram
## TurtleBot3 Waffle Food Delivery Robot

Render this file in any Mermaid-compatible viewer:
- VS Code: install "Markdown Preview Mermaid Support" extension
- GitHub: renders automatically in `.md` files
- Online: https://mermaid.live

---

## Full FSM Diagram

```mermaid
stateDiagram-v2
    [*] --> IDLE : System Boot

    IDLE --> NAVIGATING : task_received\n/delivery_task topic\n[Entry: send Nav2 goal]

    NAVIGATING --> DELIVERING_FOOD : near_goal\n[dist_to_goal ≤ 3.0m]\n[Entry: set speed=0.15m/s\nresend move_base goal]

    NAVIGATING --> IDLE : nav_timeout\n[elapsed > 120s]\n[Entry: cancel goal\nstop robot]

    DELIVERING_FOOD --> AVOIDING_OBSTACLE : obstacle_detected\n[LiDAR dist_min < 0.5m]\n[Entry: cancel move_base\nset speed=0.05m/s\nrecord avoidance_dir]

    DELIVERING_FOOD --> TASK_COMPLETED : goal_reached\n[position_error ≤ 0.1m\nAND yaw_error ≤ 5°]\n[Entry: stop robot\npublish /finish_task=True]

    AVOIDING_OBSTACLE --> DELIVERING_FOOD : path_clear_2s\n[no obstacle for ≥2.0s]\n[Entry: restore speed=0.15m/s\nresend move_base goal]

    AVOIDING_OBSTACLE --> DELIVERING_FOOD : avoidance_timeout\n[elapsed > 30s]\n[Entry: force resume\nresend goal]

    TASK_COMPLETED --> IDLE : task_confirmed\n[after 1s delay]\n[Entry: reset all state\nwait for next task]
```

---

## Sensor Fusion Flow Diagram

```mermaid
flowchart TD
    subgraph SENSORS["Sensor Layer (10ms cycle)"]
        A[RPLIDAR A1M8\n/scan @ 5-10Hz]
        B[RGB Camera\nSLAM Map Source]
    end

    subgraph PROCESSING["Processing Layer"]
        C[obstacle_detector.py\nFilter 0.5m threshold\nSector: front/left/right]
        D[AMCL Localization\n/amcl_pose\nMap + LiDAR fusion]
    end

    subgraph FSM["FSM Decision Layer (100ms cycle)"]
        E{obstacle_detected?\ndist_min < 0.5m}
        F{goal_reached?\nerror ≤ 0.1m + 5°}
        G{path clear?\n≥ 2.0s no obstacle}
    end

    subgraph ACTUATOR["Actuator Layer"]
        H[move_base\nDWA Local Planner\nGlobal NavFn]
        I[/cmd_vel\nTwist commands]
    end

    A -->|LaserScan| C
    B -->|Map tiles| D
    C -->|/obstacle_detected\n/min_obstacle_dist| E
    D -->|pose estimate| F
    E -->|YES: trigger AVOIDING| I
    E -->|NO: continue| H
    F -->|YES: trigger COMPLETED| I
    G -->|YES: trigger DELIVERING| H
    H -->|velocity| I
```

---

## State Action Table

| State | Entry Action | Loop Action | Exit Action |
|-------|-------------|-------------|-------------|
| **IDLE** | Stop robot (`/cmd_vel = 0`) | Poll `/delivery_task` | Save goal pose |
| **NAVIGATING** | Send move_base goal at 0.15m/s | Check dist_to_goal, nav timeout | — |
| **DELIVERING_FOOD** | Resend move_base goal at 0.15m/s | Monitor LiDAR + AMCL | Preserve saved goal |
| **AVOIDING_OBSTACLE** | Cancel move_base, set 0.05m/s | Publish avoidance Twist cmd | Stop robot, reset timers |
| **TASK_COMPLETED** | Stop robot, publish `/finish_task=True` | Wait 1s | Reset current_goal |

---

## Topic & Service Map

```mermaid
graph LR
    subgraph External
        T1["/delivery_task\nstd_msgs/String"]
        T2["/scan\nsensor_msgs/LaserScan"]
        T3["/amcl_pose\nPoseWithCovarianceStamped"]
    end

    subgraph FSM_Node["fsm_core.py"]
        FSM[FSM Controller\n10Hz loop]
    end

    subgraph Outputs
        T4["/robot_state\nstd_msgs/String"]
        T5["/cmd_vel\ngeometry_msgs/Twist"]
        T6["/finish_task\nstd_msgs/Bool"]
        T7["move_base\nMoveBaseAction"]
    end

    T1 -->|task command| FSM
    T2 -->|obstacle data| FSM
    T3 -->|localization| FSM
    FSM --> T4
    FSM --> T5
    FSM --> T6
    FSM --> T7
```

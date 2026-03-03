# Obstacle Avoidance Algorithm Design
## RPLIDAR A1M8 — 0.5m Threshold Adaptation

---

## 1. Hardware Characterization: RPLIDAR A1M8

| Parameter | Value | Impact on Algorithm |
|-----------|-------|-------------------|
| Scan frequency | 5.5–10 Hz | FSM must run at ≥10Hz to catch every frame |
| Angular resolution | ~1° (360 samples/scan) | Reliable sector separation at ±30° |
| Minimum valid range | 0.15m (hardware) | Algorithm uses 0.12m filter floor for safety |
| Maximum reliable range | 12m (spec: 6–12m) | Obstacle marking cut at 2.5m in costmap |
| Scan topic | `/scan` (LaserScan) | Full 360° array, `angle_min ≈ -π` |

---

## 2. Why 0.5m Is the Correct Threshold

### Physics Calculation

```
TurtleBot3 Waffle max speed:  v_max = 0.26 m/s  (hardware limit)
Delivery speed:               v_del = 0.15 m/s
Avoidance speed:              v_avd = 0.05 m/s
Mechanical braking distance:  d_brake ≈ 0.15m  (estimated from Waffle inertia)

Safety margin at 0.15m/s:
  0.5m (threshold) - 0.15m (brake) = 0.35m buffer

Time available for FSM to react:
  t_react = 0.35m / 0.15m/s = 2.33 seconds

FSM loop period = 0.1s → 23 decision cycles within safety window ✓

At avoidance speed (0.05 m/s):
  Time to reach 0.12m (minimum sensor range) = (0.5 - 0.12) / 0.05 = 7.6 seconds
  → Ample time for complete avoidance maneuver ✓
```

### Comparison with Alternative Thresholds

| Threshold | Buffer at 0.15m/s | Verdict |
|-----------|------------------|---------|
| 0.3m | 0.15m = 1.0s | Too tight, high collision risk |
| **0.5m** | **0.35m = 2.33s** | **Optimal: safe + responsive** |
| 1.0m | 0.85m = 5.7s | Too conservative, frequent false triggers |

---

## 3. Scan Sector Definitions

```
                     FRONT ±30°
                    ┌─────────┐
              LEFT  │         │  RIGHT
             30-90° │ ROBOT ► │  -30 to -90°
                    │         │
                    └─────────┘
                     REAR ±150-180°

Obstacle trigger: ANY reading in ±180° < 0.5m
Direction select: Compare dist_left vs dist_right
                  → Turn toward the larger (clearer) side
```

---

## 4. Avoidance Algorithm Phases

### Phase A — Obstacle Present (dist_min < 0.5m)

```python
twist.linear.x  = 0.05   # avoidance_speed — slow forward progress
twist.angular.z = 0.40 * avoidance_direction  # rotate toward free space
```

Geometry: At angular_z = 0.40 rad/s, robot completes 90° turn in ~3.9s.
During that time it advances: 0.05 × 3.9 = 0.195m along a curved arc.
Arc radius ≈ v/ω = 0.05/0.40 = 0.125m — tight enough to detour around a human leg.

### Phase B — Path Clearing (0.5m < dist_min, timer < 2s)

```python
twist.linear.x  = 0.05                              # maintain slow speed
twist.angular.z = 0.12 * (-avoidance_direction)     # counter-rotate back
```

This creates a smooth S-curve that returns the robot toward its original heading
without sharp reversals that could destabilize cargo.

### Phase C — Return to Delivery (timer ≥ 2.0s clear)

FSM transitions back to DELIVERING_FOOD:
1. `_cancel_nav()` → ensures move_base is clean
2. `_send_nav_goal(current_goal, speed=0.15)` → resubmit original target
3. Nav2 NavFn global planner recomputes optimal path from new position
4. AMCL pose update ensures no dead-reckoning drift from avoidance maneuver

---

## 5. No-Drift Guarantee After Detour

The critical property is that **the robot never changes its target goal**.
`current_goal` is set once when the task is received and preserved throughout
the entire avoidance episode.

```
DELIVERING → AVOIDING:   current_goal = saved (not modified)
AVOIDING   → DELIVERING: send_nav_goal(current_goal)  ← same goal
                          Nav2 plans fresh path from current AMCL position
                          AMCL corrects for any odometry drift during avoidance
```

This means even if the avoidance maneuver moves the robot 0.5m off the original
path, Nav2 + AMCL will route it back to the correct delivery point with no manual
correction required.

---

## 6. Edge Cases Handled

| Scenario | Detection | Response |
|----------|-----------|----------|
| Pedestrian walks in front | `dist_front < 0.5m` | Pause and wait, resume after 2s clear |
| Pedestrian passes behind | `dist_rear < 0.5m` but front clear | No avoidance triggered (rear only) |
| Narrow corridor (both sides blocked) | `dist_left < 0.4m AND dist_right < 0.4m` | Turn left (default) at reduced angular speed |
| LiDAR disconnects | `scan_age > 3.0s` | Emergency stop, treat as obstacle present |
| Avoidance takes too long | `avoidance_elapsed > 30s` | Force resume delivery, log error |
| Robot oscillates between states | 2s hysteresis on `clear_duration` | Prevents chattering at obstacle boundary |

---

## 7. Costmap Inflation Alignment

The `inflation_radius: 0.50m` in `costmap_params.yaml` matches the LiDAR threshold exactly:

```
Physical obstacle
       │
       ├── 0.17m ── robot_radius boundary (collision)
       ├── 0.50m ── inflation_radius boundary (costmap lethal)
       └── 0.50m ── FSM obstacle_threshold (avoidance trigger)
```

This alignment ensures:
1. DWA local planner will never route through the inflated zone
2. FSM avoidance triggers at the same distance the costmap considers lethal
3. After avoidance, Nav2 re-plans a path guaranteed to stay ≥0.5m from all obstacles

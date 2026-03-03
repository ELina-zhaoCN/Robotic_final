#!/usr/bin/env python3
"""
obstacle_detector.py — RPLIDAR A1M8 Obstacle Detection Node
============================================================
Subscribes: /scan (sensor_msgs/LaserScan)
Publishes:
  /obstacle_status  (turtlebot3_delivery/RobotState) — obstacle fields only
  /min_obstacle_dist (std_msgs/Float32)               — minimum distance in front

Parse rate: 100Hz subscriber queue + 10ms timer
Threshold:  0.5m (configurable via ROS param)

Run standalone:
  rosrun turtlebot3_delivery obstacle_detector.py

Dependencies:
  sudo apt install ros-noetic-sensor-msgs ros-noetic-std-msgs
"""

import math
import time
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool


class ObstacleDetector:
    """
    Parses RPLIDAR A1M8 /scan data every 10ms.
    Detects obstacles within 0.5m across full ±180° sweep.
    Provides per-sector distance data for FSM direction decisions.
    """

    def __init__(self):
        rospy.init_node('obstacle_detector', anonymous=False)

        # ── Parameters (overridable from launch/rosparam) ──────────────────
        self.obstacle_threshold    = rospy.get_param('~obstacle_threshold',    0.50)
        self.scan_angle_min_deg    = rospy.get_param('~scan_angle_min_deg',  -180.0)
        self.scan_angle_max_deg    = rospy.get_param('~scan_angle_max_deg',   180.0)
        self.lidar_min_valid_range = rospy.get_param('~lidar_min_valid_range', 0.12)
        self.lidar_max_valid_range = rospy.get_param('~lidar_max_valid_range', 12.0)

        # ── Internal state ──────────────────────────────────────────────────
        self.latest_scan       = None
        self.scan_received     = False
        self.last_scan_time    = None
        self.scan_timeout_secs = 3.0       # Alert if no scan for 3s

        # Per-sector minimum distances (meters), inf = no obstacle
        self.dist_front  = float('inf')    # ±30°   forward cone
        self.dist_left   = float('inf')    # 30°–90°
        self.dist_right  = float('inf')    # -90°–-30°
        self.dist_rear   = float('inf')    # ±150°–±180°
        self.dist_min_all = float('inf')   # Global minimum across all sectors

        self.obstacle_detected = False

        # ── Subscribers ─────────────────────────────────────────────────────
        self.scan_sub = rospy.Subscriber(
            '/scan', LaserScan,
            self._scan_callback,
            queue_size=10,
            buff_size=2**20
        )

        # ── Publishers ──────────────────────────────────────────────────────
        self.dist_pub     = rospy.Publisher('/min_obstacle_dist', Float32,   queue_size=5)
        self.obstacle_pub = rospy.Publisher('/obstacle_detected',  Bool,      queue_size=5)

        # Sector-specific distance topics for FSM avoidance direction logic
        self.front_pub  = rospy.Publisher('/obstacle_dist/front', Float32, queue_size=5)
        self.left_pub   = rospy.Publisher('/obstacle_dist/left',  Float32, queue_size=5)
        self.right_pub  = rospy.Publisher('/obstacle_dist/right', Float32, queue_size=5)

        # ── 10ms publish timer (matches 100Hz requirement) ──────────────────
        self.pub_timer = rospy.Timer(
            rospy.Duration(0.01),           # 10ms = 100Hz
            self._publish_callback
        )

        rospy.loginfo("[ObstacleDetector] Initialized. Threshold=%.2fm, "
                      "Scan range=[%.1f°, %.1f°]",
                      self.obstacle_threshold,
                      self.scan_angle_min_deg,
                      self.scan_angle_max_deg)

    # ── Scan callback ────────────────────────────────────────────────────────
    def _scan_callback(self, msg: LaserScan):
        """
        Process raw LaserScan from RPLIDAR A1M8.
        Filters invalid readings, computes per-sector minimums.
        Called on every incoming scan (~5–10Hz from A1M8).
        """
        self.latest_scan   = msg
        self.scan_received = True
        self.last_scan_time = time.time()
        self._process_scan(msg)

    def _process_scan(self, msg: LaserScan):
        """
        Parse scan into sector minimums.
        RPLIDAR A1M8 angle convention:
          angle_min ≈ -π, angle_max ≈ π (CCW positive)
          Index 0 = front (0°), increasing CCW
        """
        front_dists  = []
        left_dists   = []
        right_dists  = []
        rear_dists   = []
        all_dists    = []

        n = len(msg.ranges)
        if n == 0:
            rospy.logwarn_throttle(5, "[ObstacleDetector] Empty scan received.")
            return

        for i, r in enumerate(msg.ranges):
            # ── Validity filter ──────────────────────────────────────────
            if math.isnan(r) or math.isinf(r):
                continue
            if r < self.lidar_min_valid_range or r > self.lidar_max_valid_range:
                continue

            # ── Convert index to angle (degrees) ────────────────────────
            angle_rad = msg.angle_min + i * msg.angle_increment
            angle_deg = math.degrees(angle_rad)

            # Normalize to [-180, 180]
            while angle_deg >  180.0: angle_deg -= 360.0
            while angle_deg < -180.0: angle_deg += 360.0

            # ── Apply configured scan range filter ───────────────────────
            if not (self.scan_angle_min_deg <= angle_deg <= self.scan_angle_max_deg):
                continue

            all_dists.append(r)

            # ── Sector assignment ────────────────────────────────────────
            abs_deg = abs(angle_deg)

            if abs_deg <= 30.0:
                front_dists.append(r)
            elif 30.0 < abs_deg <= 90.0:
                if angle_deg > 0:
                    left_dists.append(r)
                else:
                    right_dists.append(r)
            elif abs_deg > 150.0:
                rear_dists.append(r)

        # ── Update sector minimums ────────────────────────────────────────
        self.dist_front   = min(front_dists)  if front_dists  else float('inf')
        self.dist_left    = min(left_dists)   if left_dists   else float('inf')
        self.dist_right   = min(right_dists)  if right_dists  else float('inf')
        self.dist_rear    = min(rear_dists)   if rear_dists   else float('inf')
        self.dist_min_all = min(all_dists)    if all_dists    else float('inf')

        # ── Obstacle detection (full configured scan arc) ─────────────────
        self.obstacle_detected = (self.dist_min_all < self.obstacle_threshold)

    # ── Publish timer callback (10ms) ───────────────────────────────────────
    def _publish_callback(self, event):
        """Publish current obstacle data at 100Hz."""

        # ── Scan timeout watchdog ─────────────────────────────────────────
        if self.last_scan_time is not None:
            age = time.time() - self.last_scan_time
            if age > self.scan_timeout_secs:
                rospy.logerr_throttle(
                    5,
                    "[ObstacleDetector] LIDAR TIMEOUT: no /scan for %.1fs! "
                    "Check RPLIDAR A1M8 USB connection.", age
                )
                # Treat as obstacle present for safety
                self.obstacle_detected = True
                self.dist_min_all = 0.0
                return

        if not self.scan_received:
            rospy.logwarn_throttle(
                10, "[ObstacleDetector] Waiting for first /scan message...")
            return

        # ── Publish all topics ────────────────────────────────────────────
        self.dist_pub.publish(Float32(data=self.dist_min_all))
        self.obstacle_pub.publish(Bool(data=self.obstacle_detected))
        self.front_pub.publish(Float32(data=self.dist_front))
        self.left_pub.publish(Float32(data=self.dist_left))
        self.right_pub.publish(Float32(data=self.dist_right))

        # Debug throttled log
        if self.obstacle_detected:
            rospy.logwarn_throttle(
                1,
                "[ObstacleDetector] OBSTACLE DETECTED! "
                "min_dist=%.3fm (threshold=%.2fm) | "
                "front=%.2f left=%.2f right=%.2f",
                self.dist_min_all, self.obstacle_threshold,
                self.dist_front, self.dist_left, self.dist_right
            )

    # ── Public accessors (used by fsm_core.py if co-located) ────────────────
    def get_obstacle_status(self):
        """Returns dict with current obstacle state for FSM polling."""
        return {
            'detected':   self.obstacle_detected,
            'dist_min':   self.dist_min_all,
            'dist_front': self.dist_front,
            'dist_left':  self.dist_left,
            'dist_right': self.dist_right,
            'scan_age':   (time.time() - self.last_scan_time)
                          if self.last_scan_time else float('inf')
        }

    def spin(self):
        rospy.spin()


# ── Entry point ──────────────────────────────────────────────────────────────
def main():
    try:
        detector = ObstacleDetector()
        detector.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[ObstacleDetector] Shutting down.")


if __name__ == '__main__':
    main()

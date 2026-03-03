#!/usr/bin/env python3
"""
fsm_map_fusion.py — FSM + Map + Obstacle Avoidance Master Controller
====================================================================
This is the TOP-LEVEL integration node that fuses:
  ① Restaurant map knowledge (from fsm_map_config.yaml)
  ② Real-time LiDAR obstacle detection (RPLIDAR A1M8, 0.5m threshold)
  ③ AMCL robot localization (/amcl_pose)
  ④ move_base navigation (global path + DWA local planner)

Into a unified 5-state FSM for food delivery:
  IDLE → NAVIGATING → DELIVERING_FOOD ⇄ AVOIDING_OBSTACLE → TASK_COMPLETED

DIFFERENCES from fsm_core.py:
  - Loads all delivery locations directly from fsm_map_config.yaml
  - Zone-aware navigation (kitchen / aisle / dining area phases)
  - Waypoint-based routing (passes through aisle before reaching table)
  - Richer console status display with map zone context
  - Standalone: does NOT depend on fsm_core.py

Run (after navigation.launch is up):
  rosrun turtlebot3_delivery fsm_map_fusion.py

Send delivery task:
  rostopic pub /delivery_task std_msgs/String "data: 'table_1'"

Monitor states:
  rostopic echo /robot_state

Dependencies:
  sudo apt install ros-noetic-move-base-msgs ros-noetic-dynamic-reconfigure
"""

import os
import math
import time
import threading
import yaml
from enum import IntEnum

import rospy
import actionlib
import tf

from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import (
    PoseStamped, PoseWithCovarianceStamped,
    Twist, Point, Quaternion
)
from std_msgs.msg       import String, Bool, Float32
from nav_msgs.msg       import OccupancyGrid
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

try:
    from dynamic_reconfigure.client import Client as DynReconfigClient
    DYN_OK = True
except ImportError:
    DYN_OK = False

try:
    from visualization_msgs.msg import Marker, MarkerArray
    from std_msgs.msg import ColorRGBA
    VIZ_OK = True
except ImportError:
    VIZ_OK = False


# ── FSM State Definitions ─────────────────────────────────────────────────────
class State(IntEnum):
    IDLE              = 0
    NAVIGATING        = 1
    DELIVERING_FOOD   = 2
    AVOIDING_OBSTACLE = 3
    TASK_COMPLETED    = 4

STATE_NAMES = {
    State.IDLE:              "IDLE",
    State.NAVIGATING:        "NAVIGATING",
    State.DELIVERING_FOOD:   "DELIVERING_FOOD",
    State.AVOIDING_OBSTACLE: "AVOIDING_OBSTACLE",
    State.TASK_COMPLETED:    "TASK_COMPLETED",
}

# State display for console (with box art)
STATE_DISPLAY = {
    State.IDLE:              "[ IDLE            ]  Waiting at kitchen",
    State.NAVIGATING:        "[ NAVIGATING      ]  Moving to delivery area",
    State.DELIVERING_FOOD:   "[ DELIVERING FOOD ]  Approaching table",
    State.AVOIDING_OBSTACLE: "[ AVOIDING OBS.   ]  Detouring around obstacle",
    State.TASK_COMPLETED:    "[ TASK COMPLETED  ]  Arrived, sending signal",
}


# ── Main FSM + Map Fusion Node ────────────────────────────────────────────────
class FSMMapFusion:
    """
    Integrates restaurant map knowledge with real-time sensor fusion
    and FSM-controlled delivery navigation.
    """

    # ── Config file path ─────────────────────────────────────────────────────
    PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    CONFIG_FILE = os.path.join(PACKAGE_DIR, "config", "fsm_map_config.yaml")

    def __init__(self):
        rospy.init_node('fsm_map_fusion', anonymous=False)

        # ── Load restaurant map config ───────────────────────────────────
        self.config    = self._load_config()
        self.locations = self._parse_locations()
        self.waypoints = self._parse_waypoints()
        self._print_loaded_locations()

        # ── Load FSM parameters from config ─────────────────────────────
        oa = self.config.get('obstacle_avoidance', {})
        ar = self.config.get('arrival', {})
        tm = self.config.get('timing', {})

        self.obstacle_threshold   = oa.get('detection_threshold', 0.50)
        self.clear_duration       = oa.get('clear_duration',       2.0)
        self.delivery_speed       = oa.get('delivery_speed',       0.15)
        self.avoidance_speed      = oa.get('avoidance_speed',      0.05)
        self.avoidance_ang_speed  = oa.get('angular_speed',        0.40)
        self.goal_tolerance_xy    = ar.get('position_tolerance',   0.10)
        self.goal_tolerance_yaw_r = math.radians(
                                    ar.get('yaw_tolerance_deg',    5.0))
        self.nav_goal_timeout     = tm.get('nav_goal_timeout',    120.0)
        self.avoidance_timeout    = tm.get('avoidance_timeout',    30.0)

        # ── FSM state variables ──────────────────────────────────────────
        self.state            = State.IDLE
        self.state_lock       = threading.Lock()
        self.state_enter_time = time.time()
        self.trigger_event    = ""

        # ── Navigation route plan ────────────────────────────────────────
        # Route = list of PoseStamped; robot follows them in order
        self.route        = []        # Full route: [waypoint, ..., final_goal]
        self.route_index  = 0         # Which waypoint we're currently targeting
        self.final_goal   = None      # PoseStamped — ultimate delivery target
        self.current_task_name = ""   # e.g. "table_1"

        # ── Sensor data ──────────────────────────────────────────────────
        self.current_pose     = None
        self.obstacle_detected = False
        self.dist_min_all     = float('inf')
        self.dist_front       = float('inf')
        self.dist_left        = float('inf')
        self.dist_right       = float('inf')
        self.clear_start_time = None
        self.last_scan_time   = None

        # ── Avoidance state ──────────────────────────────────────────────
        self.avoidance_direction   = 1      # +1=left, -1=right
        self.avoidance_start_time  = None

        # ── move_base client ─────────────────────────────────────────────
        rospy.loginfo("[FSMFusion] Connecting to move_base action server...")
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        connected = self.mb_client.wait_for_server(rospy.Duration(30.0))
        if connected:
            rospy.loginfo("[FSMFusion] move_base connected.")
        else:
            rospy.logerr("[FSMFusion] move_base not found after 30s! "
                         "Run navigation.launch first.")

        # ── DWA speed reconfigure ────────────────────────────────────────
        self.dwa_client = None
        if DYN_OK:
            try:
                self.dwa_client = DynReconfigClient(
                    '/move_base/DWAPlannerROS', timeout=5.0)
            except Exception:
                rospy.logwarn("[FSMFusion] DWA reconfigure unavailable. "
                              "Speed control via cmd_vel only.")

        # ── TF listener ──────────────────────────────────────────────────
        self.tf_listener = tf.TransformListener()

        # ── Subscribers ──────────────────────────────────────────────────
        # Primary: parse LiDAR directly for minimum latency
        rospy.Subscriber('/scan', LaserScan,
                         self._scan_cb, queue_size=10, buff_size=2**20)
        # AMCL localization
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self._amcl_cb, queue_size=5)
        # Task input
        rospy.Subscriber('/delivery_task', String,
                         self._task_cb, queue_size=5)
        # Backup obstacle inputs from obstacle_detector.py (if running)
        rospy.Subscriber('/obstacle_detected', Bool,
                         self._obs_bool_cb, queue_size=10)
        rospy.Subscriber('/min_obstacle_dist', Float32,
                         self._obs_dist_cb, queue_size=10)
        rospy.Subscriber('/obstacle_dist/left',  Float32,
                         lambda m: setattr(self, 'dist_left',  m.data), queue_size=10)
        rospy.Subscriber('/obstacle_dist/right', Float32,
                         lambda m: setattr(self, 'dist_right', m.data), queue_size=10)

        # ── Publishers ───────────────────────────────────────────────────
        self.state_pub   = rospy.Publisher('/robot_state', String, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',      Twist,  queue_size=5)
        self.finish_pub  = rospy.Publisher('/finish_task',  Bool,   queue_size=5)

        # RViz current-state marker
        if VIZ_OK:
            self.viz_pub = rospy.Publisher(
                '/fsm_status_marker', Marker, queue_size=5)

        # ── FSM 10Hz timer ────────────────────────────────────────────────
        rospy.Timer(rospy.Duration(0.1), self._fsm_loop)

        # ── Console status 1Hz timer ──────────────────────────────────────
        rospy.Timer(rospy.Duration(1.0), self._console_status)

        rospy.loginfo("[FSMFusion] Ready. Send tasks with:")
        rospy.loginfo("  rostopic pub /delivery_task std_msgs/String "
                      "\"data: 'table_1'\"")
        rospy.loginfo("[FSMFusion] obstacle_threshold=%.2fm, "
                      "delivery_speed=%.2fm/s, avoidance_speed=%.2fm/s",
                      self.obstacle_threshold,
                      self.delivery_speed, self.avoidance_speed)

    # =========================================================================
    #  CONFIG LOADING
    # =========================================================================

    def _load_config(self) -> dict:
        """Load fsm_map_config.yaml. Falls back to defaults if not found."""
        if os.path.exists(self.CONFIG_FILE):
            with open(self.CONFIG_FILE, 'r') as f:
                cfg = yaml.safe_load(f) or {}
            rospy.loginfo("[FSMFusion] Config loaded: %s", self.CONFIG_FILE)
            return cfg
        else:
            rospy.logwarn("[FSMFusion] Config not found: %s. "
                          "Using default locations.", self.CONFIG_FILE)
            return {}

    def _parse_locations(self) -> dict:
        """
        Parse location table from config.
        Returns: {name: PoseStamped}
        """
        locs_raw = self.config.get('locations', {
            'kitchen': [0.0, 0.0, 0.0],
            'table_1': [3.0, 2.0, 0.0],
            'table_2': [3.0, -2.0, 0.0],
            'table_3': [6.0, 0.0, 90.0],
            'table_4': [6.0, 3.0, -90.0],
            'table_5': [8.0, -1.5, 180.0],
        })
        locs = {}
        for name, coords in locs_raw.items():
            if len(coords) >= 2:
                locs[name] = self._make_pose(
                    float(coords[0]),
                    float(coords[1]),
                    float(coords[2]) if len(coords) > 2 else 0.0
                )
        return locs

    def _parse_waypoints(self) -> dict:
        """Parse intermediate waypoints for multi-segment routing."""
        wp_raw = self.config.get('waypoints', {})
        wps = {}
        for name, coords in wp_raw.items():
            if len(coords) >= 2:
                wps[name] = self._make_pose(
                    float(coords[0]),
                    float(coords[1]),
                    float(coords[2]) if len(coords) > 2 else 0.0
                )
        return wps

    def _print_loaded_locations(self):
        rospy.loginfo("[FSMFusion] Loaded %d delivery locations:",
                      len(self.locations))
        for name, pose in self.locations.items():
            x = pose.pose.position.x
            y = pose.pose.position.y
            rospy.loginfo("  %-12s → (%.2f, %.2f)", name, x, y)

    # =========================================================================
    #  SUBSCRIBER CALLBACKS
    # =========================================================================

    def _scan_cb(self, msg: LaserScan):
        """
        Parse RPLIDAR A1M8 scan data.
        Extracts per-sector minimum distances and triggers obstacle flag.
        Called at ~5–10Hz by LiDAR hardware.
        """
        self.last_scan_time = time.time()
        front_d, left_d, right_d, all_d = [], [], [], []

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < 0.12 or r > 12.0:
                continue

            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            while angle_deg >  180.0: angle_deg -= 360.0
            while angle_deg < -180.0: angle_deg += 360.0

            all_d.append(r)
            abs_a = abs(angle_deg)
            if abs_a <= 30.0:
                front_d.append(r)
            elif 30.0 < abs_a <= 90.0:
                (left_d if angle_deg > 0 else right_d).append(r)

        self.dist_front   = min(front_d)  if front_d  else float('inf')
        self.dist_left    = min(left_d)   if left_d   else float('inf')
        self.dist_right   = min(right_d)  if right_d  else float('inf')
        self.dist_min_all = min(all_d)    if all_d    else float('inf')
        self.obstacle_detected = (self.dist_min_all < self.obstacle_threshold)

        # Update clearance timer for AVOIDING → DELIVERING transition
        if not self.obstacle_detected:
            if self.clear_start_time is None:
                self.clear_start_time = time.time()
        else:
            self.clear_start_time = None

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def _task_cb(self, msg: String):
        """
        Accept delivery task.
        Accepted only in IDLE state.
        Parses: "table_1" | "x,y" | "x,y,yaw_deg"
        """
        cmd = msg.data.strip()

        if self.state != State.IDLE:
            rospy.logwarn(
                "[FSMFusion] Task '%s' rejected — robot is %s, not IDLE. "
                "Wait for current task to finish.", cmd, STATE_NAMES[self.state])
            return

        goal_pose, task_name = self._parse_task_goal(cmd)
        if goal_pose is None:
            rospy.logerr("[FSMFusion] Cannot parse task: '%s'. "
                         "Use location name or 'x,y' or 'x,y,yaw'", cmd)
            return

        self.final_goal       = goal_pose
        self.current_task_name = task_name
        self._build_route(goal_pose)

        rospy.loginfo("[FSMFusion] Task accepted: '%s' → (%.2f, %.2f)",
                      task_name,
                      goal_pose.pose.position.x,
                      goal_pose.pose.position.y)
        self._transition_to(State.NAVIGATING, trigger="task_received")

    # Backup callbacks from obstacle_detector.py
    def _obs_bool_cb(self, msg: Bool):
        if self.last_scan_time is not None:
            return   # Direct scan callback takes priority
        prev = self.obstacle_detected
        self.obstacle_detected = msg.data
        if not msg.data and prev:
            self.clear_start_time = time.time()
        elif msg.data:
            self.clear_start_time = None

    def _obs_dist_cb(self, msg: Float32):
        if self.last_scan_time is None:
            self.dist_min_all = msg.data

    # =========================================================================
    #  FSM MAIN LOOP (10Hz)
    # =========================================================================

    def _fsm_loop(self, event):
        """
        Called every 100ms by rospy.Timer.
        Evaluates all transition conditions and dispatches state handlers.
        """
        # ── LiDAR watchdog (safety critical) ────────────────────────────
        if (self.last_scan_time is not None and
                time.time() - self.last_scan_time > 3.0 and
                self.state not in (State.IDLE, State.TASK_COMPLETED)):
            rospy.logerr_throttle(
                5, "[FSMFusion] LIDAR TIMEOUT (>3s). Emergency stop!")
            self._stop_robot()
            self.obstacle_detected = True

        # ── State dispatch ───────────────────────────────────────────────
        s = self.state
        if   s == State.IDLE:              self._s_idle()
        elif s == State.NAVIGATING:        self._s_navigating()
        elif s == State.DELIVERING_FOOD:   self._s_delivering()
        elif s == State.AVOIDING_OBSTACLE: self._s_avoiding()
        elif s == State.TASK_COMPLETED:    self._s_completed()

        # ── Publish state ────────────────────────────────────────────────
        self.state_pub.publish(String(data=STATE_NAMES[self.state]))

    # =========================================================================
    #  STATE HANDLERS
    # =========================================================================

    def _s_idle(self):
        """IDLE: Robot at kitchen, stopped. Waiting for /delivery_task."""
        self._stop_robot()

    def _s_navigating(self):
        """
        NAVIGATING: Follow the pre-built route toward the delivery target.
        Uses waypoints from fsm_map_config.yaml if the route has intermediate
        stops (e.g., aisle entry point before reaching the table).
        """
        # ── Nav timeout watchdog ─────────────────────────────────────────
        if (self.route_index > 0 and          # avoid triggering before start
                time.time() - self.state_enter_time > self.nav_goal_timeout):
            rospy.logerr("[FSMFusion] Navigation TIMEOUT (%.0fs). "
                         "Returning IDLE.", self.nav_goal_timeout)
            self._cancel_nav()
            self._transition_to(State.IDLE, trigger="nav_timeout")
            return

        # ── Advance route waypoints ──────────────────────────────────────
        if self.route and self.route_index < len(self.route):
            wp = self.route[self.route_index]
            dist = self._dist_to_pose(wp)

            if dist < 0.5:    # Within 0.5m of waypoint → advance to next
                self.route_index += 1
                if self.route_index < len(self.route):
                    # Send next waypoint
                    next_wp = self.route[self.route_index]
                    self._send_nav_goal(next_wp, speed=self.delivery_speed)
                    rospy.loginfo("[FSMFusion] Waypoint %d/%d reached. "
                                  "Next: (%.2f, %.2f)",
                                  self.route_index, len(self.route),
                                  next_wp.pose.position.x,
                                  next_wp.pose.position.y)
                else:
                    # Reached final goal → switch to DELIVERING
                    rospy.loginfo("[FSMFusion] Near delivery zone. "
                                  "Switching to DELIVERING_FOOD.")
                    self._transition_to(State.DELIVERING_FOOD,
                                        trigger="near_final_goal")

    def _s_delivering(self):
        """
        DELIVERING_FOOD: Precise final approach to the table.

        FSM Sensor Fusion Point:
          ① LiDAR: if dist_min < 0.5m → AVOIDING_OBSTACLE
          ② AMCL:  if position error < 0.1m AND yaw error < 5° → TASK_COMPLETED
        """
        # ── TRANSITION ①: Obstacle detected (0.5m threshold) ─────────────
        if self.obstacle_detected:
            rospy.logwarn(
                "[FSMFusion] Obstacle at %.3fm (< %.2fm). "
                "Task: '%s'. Switching to AVOIDING.",
                self.dist_min_all, self.obstacle_threshold,
                self.current_task_name)
            # Choose direction: toward the side with more space
            self.avoidance_direction = (1 if self.dist_left >= self.dist_right
                                        else -1)
            self._transition_to(State.AVOIDING_OBSTACLE,
                                 trigger="obstacle_detected")
            return

        # ── TRANSITION ②: Goal reached ────────────────────────────────────
        if self._is_goal_reached():
            rospy.loginfo("[FSMFusion] Arrived at '%s'. dist=%.3fm",
                          self.current_task_name,
                          self._dist_to_pose(self.final_goal))
            self._transition_to(State.TASK_COMPLETED, trigger="goal_reached")

    def _s_avoiding(self):
        """
        AVOIDING_OBSTACLE: Local obstacle avoidance maneuver.

        3-Phase strategy:
          A. Obstacle present → stop forward progress, rotate to clear side
          B. Obstacle gone, completing arc → slow forward + counter-turn
          C. Clear for 2s → resume delivery
        """
        # ── Avoidance timeout watchdog ────────────────────────────────────
        if (self.avoidance_start_time and
                time.time() - self.avoidance_start_time > self.avoidance_timeout):
            rospy.logerr("[FSMFusion] Avoidance TIMEOUT (%.0fs). "
                         "Force-resuming delivery.", self.avoidance_timeout)
            self._transition_to(State.DELIVERING_FOOD, trigger="avoidance_timeout")
            return

        # ── TRANSITION: 2s consecutive clearance ──────────────────────────
        if (self.clear_start_time and
                time.time() - self.clear_start_time >= self.clear_duration):
            rospy.loginfo("[FSMFusion] Path clear for %.1fs. "
                          "Resuming '%s' delivery.",
                          self.clear_duration, self.current_task_name)
            self._transition_to(State.DELIVERING_FOOD, trigger="path_clear_2s")
            return

        # ── Publish avoidance velocity ────────────────────────────────────
        self.cmd_vel_pub.publish(self._avoidance_twist())

    def _s_completed(self):
        """
        TASK_COMPLETED: Stop, signal completion, return to IDLE.
        """
        self._stop_robot()
        self._cancel_nav()
        self.finish_pub.publish(Bool(data=True))
        rospy.loginfo("[FSMFusion] /finish_task=True published. Task '%s' done.",
                      self.current_task_name)
        rospy.sleep(1.5)   # Brief pause for physical delivery
        self._transition_to(State.IDLE, trigger="task_confirmed")

    # =========================================================================
    #  AVOIDANCE VELOCITY
    # =========================================================================

    def _avoidance_twist(self) -> Twist:
        """
        Compute velocity for avoidance phase.
        speed: 0.05 m/s (avoidance_speed from config)
        angular: 0.40 rad/s × direction

        Phase A (obstacle present): rotate in place + creep
        Phase B (clearing): slow forward + gentle counter-rotation back to path
        """
        twist = Twist()
        if self.obstacle_detected:
            # Phase A: Active avoidance — rotate toward open space
            twist.linear.x  = self.avoidance_speed          # 0.05 m/s
            twist.angular.z = (self.avoidance_ang_speed
                               * self.avoidance_direction)   # ±0.40 rad/s
        else:
            # Phase B: Path clearing — arc back toward original heading
            twist.linear.x  = self.avoidance_speed
            twist.angular.z = (self.avoidance_ang_speed * 0.3
                               * (-self.avoidance_direction))
        return twist

    # =========================================================================
    #  STATE TRANSITION MANAGER
    # =========================================================================

    def _transition_to(self, new_state: State, trigger: str = ""):
        with self.state_lock:
            if self.state == new_state:
                return
            old_state = self.state

            rospy.loginfo(
                "[FSMFusion] ═══ %s → %s  [%s] ═══",
                STATE_NAMES[old_state], STATE_NAMES[new_state], trigger)
            self.trigger_event = trigger

            # ── Exit actions ──────────────────────────────────────────────
            if old_state == State.AVOIDING_OBSTACLE:
                self.avoidance_start_time = None
                self.clear_start_time     = None
                self._stop_robot()

            # ── Update state ───────────────────────────────────────────────
            self.state            = new_state
            self.state_enter_time = time.time()

            # ── Entry actions ──────────────────────────────────────────────
            if new_state == State.IDLE:
                self._stop_robot()
                self.final_goal        = None
                self.current_task_name = ""
                self.route             = []
                self.route_index       = 0

            elif new_state == State.NAVIGATING:
                if self.route:
                    # Send the first route waypoint
                    self.route_index = 0
                    self._send_nav_goal(self.route[0], speed=self.delivery_speed)

            elif new_state == State.DELIVERING_FOOD:
                # Re-send final goal after avoidance or after waypoints complete
                if self.final_goal:
                    self._send_nav_goal(self.final_goal,
                                        speed=self.delivery_speed)
                    self._set_dwa_speed(self.delivery_speed)

            elif new_state == State.AVOIDING_OBSTACLE:
                self._cancel_nav()
                self._set_dwa_speed(self.avoidance_speed)
                self.avoidance_start_time = time.time()
                self.clear_start_time     = None

            elif new_state == State.TASK_COMPLETED:
                self._stop_robot()
                self._cancel_nav()

    # =========================================================================
    #  ROUTE PLANNING
    # =========================================================================

    def _build_route(self, final_goal: PoseStamped):
        """
        Build an ordered list of waypoints from kitchen → table.
        If waypoints are defined in config (e.g., aisle entry),
        insert them before the final goal to guide through the aisle.
        """
        self.route       = []
        self.route_index = 0

        # Add intermediate waypoints if defined and relevant
        # (Only add if waypoint is "between" kitchen and goal)
        if 'to_dining_area' in self.waypoints:
            wp = self.waypoints['to_dining_area']
            goal_x = final_goal.pose.position.x
            # Only use aisle waypoint if goal is in the dining area (x > 2m)
            if goal_x > 2.0:
                self.route.append(wp)

        if 'aisle_mid' in self.waypoints:
            wp = self.waypoints['aisle_mid']
            goal_x = final_goal.pose.position.x
            if goal_x > 4.0:   # Only if goal is deep in dining area
                self.route.append(wp)

        # Always append the final goal
        self.route.append(final_goal)

        rospy.loginfo("[FSMFusion] Route built: %d waypoints → final goal "
                      "(%.2f, %.2f)",
                      len(self.route) - 1,
                      final_goal.pose.position.x,
                      final_goal.pose.position.y)

    # =========================================================================
    #  NAVIGATION HELPERS
    # =========================================================================

    def _send_nav_goal(self, goal_pose: PoseStamped, speed: float = None):
        """Send a goal to move_base ActionServer."""
        if speed is not None:
            self._set_dwa_speed(speed)

        mb_goal = MoveBaseGoal()
        mb_goal.target_pose = goal_pose
        mb_goal.target_pose.header.frame_id = 'map'
        mb_goal.target_pose.header.stamp    = rospy.Time.now()

        self.mb_client.send_goal(
            mb_goal,
            done_cb=self._nav_done_cb
        )
        rospy.loginfo("[FSMFusion] Nav goal → (%.3f, %.3f) speed=%.2fm/s",
                      goal_pose.pose.position.x,
                      goal_pose.pose.position.y,
                      speed if speed else self.delivery_speed)

    def _cancel_nav(self):
        self.mb_client.cancel_all_goals()

    def _nav_done_cb(self, status, result):
        status_str = {
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED:   "ABORTED",
            GoalStatus.PREEMPTED: "PREEMPTED",
        }.get(status, f"CODE_{status}")
        rospy.loginfo("[FSMFusion] move_base: %s", status_str)

    def _set_dwa_speed(self, speed: float):
        speed = max(0.02, min(speed, 0.22))
        if self.dwa_client is not None:
            try:
                self.dwa_client.update_configuration({
                    'max_vel_x':     speed,
                    'max_vel_trans': speed
                })
            except Exception:
                pass

    def _stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    # =========================================================================
    #  GOAL ARRIVAL CHECK
    # =========================================================================

    def _is_goal_reached(self) -> bool:
        """Returns True if within 0.1m position AND 5° yaw of final_goal."""
        if self.current_pose is None or self.final_goal is None:
            return False
        if self._dist_to_pose(self.final_goal) > self.goal_tolerance_xy:
            return False

        q_c = self.current_pose.orientation
        q_g = self.final_goal.pose.orientation
        _, _, yaw_c = tf.transformations.euler_from_quaternion(
            [q_c.x, q_c.y, q_c.z, q_c.w])
        _, _, yaw_g = tf.transformations.euler_from_quaternion(
            [q_g.x, q_g.y, q_g.z, q_g.w])
        yaw_err = abs(self._wrap_angle(yaw_c - yaw_g))
        return yaw_err <= self.goal_tolerance_yaw_r

    def _dist_to_pose(self, pose: PoseStamped) -> float:
        if self.current_pose is None:
            return float('inf')
        dx = self.current_pose.position.x - pose.pose.position.x
        dy = self.current_pose.position.y - pose.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    # =========================================================================
    #  TASK PARSER
    # =========================================================================

    def _parse_task_goal(self, cmd: str):
        """
        Returns (PoseStamped, task_name) or (None, None).
        Accepts: "table_1" | "x,y" | "x,y,yaw_deg"
        """
        if cmd in self.locations:
            return self.locations[cmd], cmd

        parts = [p.strip() for p in cmd.split(',')]
        if len(parts) in (2, 3):
            try:
                x   = float(parts[0])
                y   = float(parts[1])
                yaw = float(parts[2]) if len(parts) == 3 else 0.0
                return self._make_pose(x, y, yaw), f"coord({x:.1f},{y:.1f})"
            except ValueError:
                pass
        return None, None

    # =========================================================================
    #  CONSOLE STATUS DISPLAY
    # =========================================================================

    def _console_status(self, event):
        """Print a one-line status summary every second."""
        dist_str = (f"{self.dist_min_all:.2f}m"
                    if self.dist_min_all < float('inf') else "N/A")
        obs_str  = "OBSTACLE!" if self.obstacle_detected else "clear"
        task_str = self.current_task_name or "—"

        rospy.loginfo_throttle(
            5,
            "[FSMFusion] %s | task=%s | lidar_min=%s (%s) | "
            "dist_to_goal=%.2fm",
            STATE_DISPLAY[self.state],
            task_str,
            dist_str,
            obs_str,
            self._dist_to_pose(self.final_goal)
            if self.final_goal else 0.0
        )

    # =========================================================================
    #  UTILITIES
    # =========================================================================

    @staticmethod
    def _make_pose(x: float, y: float, yaw_deg: float = 0.0) -> PoseStamped:
        q = tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw_deg))
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = rospy.Time.now()
        pose.pose.position   = Point(x=x, y=y, z=0.0)
        pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    @staticmethod
    def _wrap_angle(a: float) -> float:
        while a >  math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a

    def spin(self):
        rospy.spin()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    try:
        node = FSMMapFusion()
        node.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[FSMFusion] Shutdown.")


if __name__ == "__main__":
    main()

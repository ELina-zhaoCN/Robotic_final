#!/usr/bin/env python3
"""
fsm_core.py — Main FSM Controller for TurtleBot3 Waffle Food Delivery
======================================================================
Implements the 5-state Finite State Machine:

  IDLE → NAVIGATING → DELIVERING_FOOD ⇄ AVOIDING_OBSTACLE → TASK_COMPLETED → IDLE

State transition loop runs at 10Hz via rospy.Timer(0.1s).

Subscribes:
  /scan               (sensor_msgs/LaserScan)                — RPLIDAR A1M8
  /amcl_pose          (geometry_msgs/PoseWithCovarianceStamped) — AMCL localization
  /delivery_task      (std_msgs/String)                      — incoming task commands
  /obstacle_detected  (std_msgs/Bool)                        — from obstacle_detector
  /min_obstacle_dist  (std_msgs/Float32)                     — min LiDAR distance
  /obstacle_dist/front|left|right (std_msgs/Float32)         — sector distances

Publishes:
  /robot_state        (std_msgs/String)                      — current FSM state name
  /cmd_vel            (geometry_msgs/Twist)                  — velocity commands
  /finish_task        (std_msgs/Bool)                        — task completion signal

Actions:
  move_base           (move_base_msgs/MoveBaseAction)        — nav goals

Run standalone (after navigation.launch):
  rosrun turtlebot3_delivery fsm_core.py

Send a task:
  rostopic pub /delivery_task std_msgs/String "data: 'table_1'"
"""

import math
import time
import threading
from enum import IntEnum

import rospy
import actionlib
import tf

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import (
    PoseStamped, PoseWithCovarianceStamped, Twist, Point, Quaternion
)
from std_msgs.msg import String, Bool, Float32
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

try:
    from dynamic_reconfigure.client import Client as DynClient
    DYN_RECONFIG_AVAILABLE = True
except ImportError:
    DYN_RECONFIG_AVAILABLE = False


# ── FSM State Enum ────────────────────────────────────────────────────────────
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


# ── Main FSM Node ─────────────────────────────────────────────────────────────
class FoodDeliveryFSM:
    """
    Central FSM controller.
    All sensor fusion and state transition logic lives here.
    """

    def __init__(self):
        rospy.init_node('fsm_core', anonymous=False)

        # ── Load parameters ────────────────────────────────────────────────
        self.obstacle_threshold     = rospy.get_param('~obstacle_threshold',     0.50)
        self.clear_duration         = rospy.get_param('~clear_duration',         2.0)
        self.goal_tolerance_xy      = rospy.get_param('~goal_tolerance_xy',      0.10)
        goal_tolerance_yaw_deg      = rospy.get_param('~goal_tolerance_yaw_deg', 5.0)
        self.goal_tolerance_yaw_rad = math.radians(goal_tolerance_yaw_deg)
        self.delivery_speed         = rospy.get_param('~delivery_speed',         0.15)
        self.avoidance_speed        = rospy.get_param('~avoidance_speed',        0.05)
        self.avoidance_ang_speed    = rospy.get_param('~avoidance_angular_speed',0.40)
        self.nav_goal_timeout       = rospy.get_param('~nav_goal_timeout',      120.0)
        self.avoidance_timeout      = rospy.get_param('~avoidance_timeout',      30.0)

        # ── FSM state variables ────────────────────────────────────────────
        self.state           = State.IDLE
        self.state_lock      = threading.Lock()
        self.state_enter_time = time.time()
        self.trigger_event   = ""

        # ── Sensor data ────────────────────────────────────────────────────
        self.obstacle_detected = False
        self.dist_min_all      = float('inf')
        self.dist_front        = float('inf')
        self.dist_left         = float('inf')
        self.dist_right        = float('inf')
        self.current_pose      = None          # geometry_msgs/Pose from AMCL
        self.scan_received     = False
        self.last_scan_time    = None

        # ── Navigation state ───────────────────────────────────────────────
        self.current_goal          = None      # PoseStamped — active target
        self.goal_sent_time        = None
        self.avoidance_start_time  = None
        self.clear_start_time      = None      # Timestamp when path cleared
        self.avoidance_direction   = 1         # +1=CCW(left), -1=CW(right)
        self.nav_active            = False

        # ── move_base Action Client ────────────────────────────────────────
        self.mb_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("[FSM] Waiting for move_base action server...")
        if not self.mb_client.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("[FSM] move_base not available! "
                         "Is navigation.launch running?")
        else:
            rospy.loginfo("[FSM] move_base connected.")

        # ── DWA speed control ──────────────────────────────────────────────
        self.dwa_client = None
        if DYN_RECONFIG_AVAILABLE:
            try:
                self.dwa_client = DynClient(
                    '/move_base/DWAPlannerROS', timeout=5.0)
            except Exception:
                rospy.logwarn("[FSM] DWA reconfigure unavailable. "
                              "Speed locked at delivery_speed.")

        # ── TF ────────────────────────────────────────────────────────────
        self.tf_listener = tf.TransformListener()

        # ── Location map (from fsm_params.yaml) ───────────────────────────
        self.location_map = rospy.get_param('/fsm/locations', {
            'kitchen': [0.0, 0.0, 0.0],
            'table_1': [3.0, 2.0, 0.0],
        })

        # ── Subscribers ───────────────────────────────────────────────────
        rospy.Subscriber('/scan', LaserScan,
                         self._scan_callback, queue_size=10, buff_size=2**20)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                         self._amcl_callback, queue_size=5)
        rospy.Subscriber('/delivery_task', String,
                         self._task_callback, queue_size=5)

        # Obstacle detector outputs (from obstacle_detector.py)
        rospy.Subscriber('/obstacle_detected', Bool,
                         self._obstacle_bool_callback, queue_size=10)
        rospy.Subscriber('/min_obstacle_dist', Float32,
                         self._dist_all_callback, queue_size=10)
        rospy.Subscriber('/obstacle_dist/front', Float32,
                         self._dist_front_callback, queue_size=10)
        rospy.Subscriber('/obstacle_dist/left', Float32,
                         self._dist_left_callback, queue_size=10)
        rospy.Subscriber('/obstacle_dist/right', Float32,
                         self._dist_right_callback, queue_size=10)

        # ── Publishers ────────────────────────────────────────────────────
        self.state_pub      = rospy.Publisher('/robot_state',  String, queue_size=5)
        self.cmd_vel_pub    = rospy.Publisher('/cmd_vel',       Twist,  queue_size=5)
        self.finish_pub     = rospy.Publisher('/finish_task',   Bool,   queue_size=5)

        # ── 10Hz FSM update timer ──────────────────────────────────────────
        self.fsm_timer = rospy.Timer(
            rospy.Duration(0.1),    # 100ms = 10Hz
            self._fsm_update_callback
        )

        rospy.loginfo(
            "[FSM] Initialized. obstacle_threshold=%.2fm, clear_duration=%.1fs, "
            "goal_tolerance=%.2fm/%.1f°, delivery_speed=%.2fm/s, avoidance_speed=%.2fm/s",
            self.obstacle_threshold, self.clear_duration,
            self.goal_tolerance_xy, goal_tolerance_yaw_deg,
            self.delivery_speed, self.avoidance_speed
        )
        rospy.loginfo("[FSM] Entering IDLE state. "
                      "Send task: rostopic pub /delivery_task "
                      "std_msgs/String \"data: 'table_1'\"")

    # =========================================================================
    #  SUBSCRIBER CALLBACKS
    # =========================================================================

    def _scan_callback(self, msg: LaserScan):
        """
        Process /scan directly in FSM for real-time obstacle detection.
        Also updates sector distances for avoidance direction selection.
        Runs at A1M8 scan rate (~5–10Hz).
        """
        self.scan_received  = True
        self.last_scan_time = time.time()

        front_d, left_d, right_d, all_d = [], [], [], []

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < 0.12 or r > 12.0:
                continue

            angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
            # Normalize to [-180, 180]
            while angle_deg >  180.0: angle_deg -= 360.0
            while angle_deg < -180.0: angle_deg += 360.0

            all_d.append(r)

            abs_a = abs(angle_deg)
            if abs_a <= 30.0:
                front_d.append(r)
            elif 30.0 < abs_a <= 90.0:
                (left_d if angle_deg > 0 else right_d).append(r)

        self.dist_front  = min(front_d)  if front_d  else float('inf')
        self.dist_left   = min(left_d)   if left_d   else float('inf')
        self.dist_right  = min(right_d)  if right_d  else float('inf')
        self.dist_min_all = min(all_d)   if all_d    else float('inf')
        self.obstacle_detected = self.dist_min_all < self.obstacle_threshold

        # Update clear-path timer
        if not self.obstacle_detected:
            if self.clear_start_time is None:
                self.clear_start_time = time.time()
        else:
            self.clear_start_time = None

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = msg.pose.pose

    def _task_callback(self, msg: String):
        """
        Accept task only when in IDLE state.
        Parses "table_1" or "x,y" or "x,y,yaw_deg" formats.
        """
        if self.state != State.IDLE:
            rospy.logwarn(
                "[FSM] Task ignored — robot is %s (not IDLE). "
                "Queue via task_manager.py.", STATE_NAMES[self.state])
            return

        goal = self._parse_goal(msg.data.strip())
        if goal is None:
            rospy.logerr("[FSM] Cannot parse task: '%s'", msg.data)
            return

        self.current_goal = goal
        rospy.loginfo("[FSM] Task accepted: (%.2f, %.2f)",
                      goal.pose.position.x, goal.pose.position.y)
        self._transition_to(State.NAVIGATING, trigger="task_received")

    # ── Obstacle detector topic callbacks (backup if scan not direct) ────────
    def _obstacle_bool_callback(self, msg: Bool):
        if self.scan_received:
            return   # Scan callback takes precedence
        self.obstacle_detected = msg.data
        if not msg.data:
            if self.clear_start_time is None:
                self.clear_start_time = time.time()
        else:
            self.clear_start_time = None

    def _dist_all_callback(self,   msg: Float32): self.dist_min_all = msg.data
    def _dist_front_callback(self, msg: Float32): self.dist_front   = msg.data
    def _dist_left_callback(self,  msg: Float32): self.dist_left    = msg.data
    def _dist_right_callback(self, msg: Float32): self.dist_right   = msg.data

    # =========================================================================
    #  FSM MAIN LOOP (10Hz)
    # =========================================================================

    def _fsm_update_callback(self, event):
        """Evaluate state transitions and execute state actions every 100ms."""

        # ── Watchdog: LiDAR timeout ──────────────────────────────────────────
        if (self.last_scan_time is not None and
                time.time() - self.last_scan_time > 3.0):
            rospy.logerr_throttle(
                5, "[FSM] LiDAR TIMEOUT (>3s no /scan). "
                "Treating as obstacle present for safety.")
            self.obstacle_detected = True
            self.dist_min_all = 0.0

        # ── Dispatch to state handler ────────────────────────────────────────
        with self.state_lock:
            current = self.state

        if   current == State.IDLE:              self._handle_idle()
        elif current == State.NAVIGATING:        self._handle_navigating()
        elif current == State.DELIVERING_FOOD:   self._handle_delivering()
        elif current == State.AVOIDING_OBSTACLE: self._handle_avoiding()
        elif current == State.TASK_COMPLETED:    self._handle_completed()

        # ── Publish current state ────────────────────────────────────────────
        self.state_pub.publish(String(data=STATE_NAMES[self.state]))

    # =========================================================================
    #  STATE HANDLERS
    # =========================================================================

    def _handle_idle(self):
        """IDLE: Robot at kitchen base, waiting for task. Stop all motion."""
        self._stop_robot()

    def _handle_navigating(self):
        """
        NAVIGATING: Move from kitchen to delivery target area via move_base.
        Transitions to DELIVERING_FOOD when within 3m of goal.
        """
        # ── Navigation timeout watchdog ──────────────────────────────────────
        if (self.goal_sent_time is not None and
                time.time() - self.goal_sent_time > self.nav_goal_timeout):
            rospy.logerr("[FSM] Navigation TIMEOUT after %.0fs. "
                         "Returning to IDLE.", self.nav_goal_timeout)
            self._cancel_nav()
            self._transition_to(State.IDLE, trigger="nav_timeout")
            return

        # ── Check if close enough to switch to delivery mode ─────────────────
        dist_to_goal = self._distance_to_goal()
        if dist_to_goal <= 3.0:
            rospy.loginfo("[FSM] Within 3m of goal (%.2fm). "
                          "Switching to DELIVERING_FOOD.", dist_to_goal)
            self._transition_to(State.DELIVERING_FOOD, trigger="near_goal")

    def _handle_delivering(self):
        """
        DELIVERING_FOOD: Precise path tracking toward target dining table.

        Sensor fusion point:
          - LiDAR (/scan) triggers → AVOIDING_OBSTACLE if dist < 0.5m
          - AMCL (/amcl_pose) triggers → TASK_COMPLETED if within 0.1m + 5°
        """
        # ── TRANSITION 1: Obstacle detected (LiDAR < 0.5m) ──────────────────
        if self.obstacle_detected:
            rospy.logwarn(
                "[FSM] OBSTACLE at %.3fm (< %.2fm threshold). "
                "Triggering AVOIDANCE.", self.dist_min_all, self.obstacle_threshold)
            # Select avoidance direction: go toward the side with more space
            self.avoidance_direction = 1 if self.dist_left >= self.dist_right else -1
            dir_name = "LEFT" if self.avoidance_direction == 1 else "RIGHT"
            rospy.loginfo("[FSM] Avoidance direction: %s "
                          "(left=%.2fm, right=%.2fm)",
                          dir_name, self.dist_left, self.dist_right)
            self._transition_to(State.AVOIDING_OBSTACLE, trigger="obstacle_detected")
            return

        # ── TRANSITION 2: Goal reached ────────────────────────────────────────
        if self._is_goal_reached():
            rospy.loginfo("[FSM] GOAL REACHED within tolerance "
                          "(%.3fm, yaw within 5°).", self._distance_to_goal())
            self._transition_to(State.TASK_COMPLETED, trigger="goal_reached")

    def _handle_avoiding(self):
        """
        AVOIDING_OBSTACLE: Execute local obstacle avoidance maneuver.

        Algorithm (simplified VFH with safe distance envelope):
          Phase A — Obstacle present:
            Linear x = avoidance_speed (0.05 m/s), rotate away from obstacle
          Phase B — Path clear, still completing detour:
            Slow forward + corrective turn back toward original path
          Phase C — 2 consecutive seconds clear → resume delivery

        The 0.5m detection margin with 0.05m/s approach speed provides
        ≥10 seconds of reaction time before any collision could occur.
        """
        # ── Avoidance timeout watchdog ────────────────────────────────────────
        if (self.avoidance_start_time is not None and
                time.time() - self.avoidance_start_time > self.avoidance_timeout):
            rospy.logerr("[FSM] Avoidance TIMEOUT after %.0fs. "
                         "Resuming delivery anyway.", self.avoidance_timeout)
            self._transition_to(State.DELIVERING_FOOD, trigger="avoidance_timeout")
            return

        # ── TRANSITION: 2 consecutive seconds with no obstacle ───────────────
        if (self.clear_start_time is not None and
                time.time() - self.clear_start_time >= self.clear_duration):
            rospy.loginfo(
                "[FSM] Path clear for %.1fs (>= %.1fs). "
                "Resuming delivery.", self.clear_duration, self.clear_duration)
            self._transition_to(State.DELIVERING_FOOD, trigger="path_clear_2s")
            return

        # ── Avoidance velocity command ────────────────────────────────────────
        twist = self._compute_avoidance_twist()
        self.cmd_vel_pub.publish(twist)

    def _handle_completed(self):
        """
        TASK_COMPLETED: Stop robot, send /finish_task=True, return to IDLE.
        """
        self._stop_robot()
        self._cancel_nav()
        self.finish_pub.publish(Bool(data=True))
        rospy.loginfo("[FSM] Task complete signal published to /finish_task.")
        rospy.sleep(1.0)
        self._transition_to(State.IDLE, trigger="task_confirmed")

    # =========================================================================
    #  AVOIDANCE MOTION CONTROLLER
    # =========================================================================

    def _compute_avoidance_twist(self) -> Twist:
        """
        Compute velocity command for obstacle avoidance phase.

        Strategy:
          1. If front obstacle < 0.5m: stop forward motion, rotate away
          2. If front clear but side obstacle: move forward slowly + slight turn
          3. Both sides clear after detour: move forward + straighten

        All motion capped at avoidance_speed (0.05 m/s) as specified.
        """
        twist = Twist()

        if self.obstacle_detected:
            # ── Phase A: Active obstacle — stop and turn ──────────────────
            # Linear: very slow crawl (0.05 m/s) to make progress during turn
            # Angular: rotate toward the selected free side
            twist.linear.x  = self.avoidance_speed  # 0.05 m/s
            twist.angular.z = self.avoidance_ang_speed * self.avoidance_direction
        else:
            # ── Phase B: Obstacle cleared, completing detour arc ──────────
            # Move forward at avoidance speed, gentle counter-rotation
            # to arc back toward the original path heading
            twist.linear.x  = self.avoidance_speed
            twist.angular.z = (self.avoidance_ang_speed * 0.3
                                * (-self.avoidance_direction))

        return twist

    # =========================================================================
    #  STATE TRANSITION MANAGER
    # =========================================================================

    def _transition_to(self, new_state: State, trigger: str = ""):
        """
        Execute state exit actions → update state → execute entry actions.
        Thread-safe via state_lock.
        """
        with self.state_lock:
            old_state = self.state
            if old_state == new_state:
                return

            rospy.loginfo("[FSM] TRANSITION: %s → %s  [trigger: %s]",
                          STATE_NAMES[old_state], STATE_NAMES[new_state], trigger)
            self.trigger_event = trigger

            # ── Exit actions ─────────────────────────────────────────────────
            self._on_exit(old_state)

            self.state = new_state
            self.state_enter_time = time.time()

            # ── Entry actions ─────────────────────────────────────────────────
            self._on_enter(new_state)

    def _on_exit(self, state: State):
        """Actions performed when leaving a state."""
        if state == State.DELIVERING_FOOD:
            pass   # goal preserved for resume_goal

        elif state == State.AVOIDING_OBSTACLE:
            self.avoidance_start_time = None
            self.clear_start_time     = None
            self._stop_robot()

    def _on_enter(self, state: State):
        """Actions performed when entering a state."""
        if state == State.IDLE:
            self._stop_robot()
            self.current_goal = None

        elif state == State.NAVIGATING:
            if self.current_goal:
                self._send_nav_goal(self.current_goal, speed=self.delivery_speed)
            else:
                rospy.logerr("[FSM] NAVIGATING entered but no goal set!")
                self.state = State.IDLE

        elif state == State.DELIVERING_FOOD:
            if self.current_goal:
                self._send_nav_goal(self.current_goal, speed=self.delivery_speed)
                rospy.loginfo("[FSM] Delivery speed restored: %.2fm/s",
                              self.delivery_speed)
            else:
                rospy.logerr("[FSM] DELIVERING entered but no goal!")
                self.state = State.IDLE

        elif state == State.AVOIDING_OBSTACLE:
            # Cancel move_base → take manual velocity control
            self._cancel_nav()
            self._set_dwa_speed(self.avoidance_speed)
            self.avoidance_start_time = time.time()
            self.clear_start_time     = None
            rospy.loginfo("[FSM] Avoidance speed set: %.2fm/s",
                          self.avoidance_speed)

        elif state == State.TASK_COMPLETED:
            self._stop_robot()

    # =========================================================================
    #  NAVIGATION HELPERS
    # =========================================================================

    def _send_nav_goal(self, goal_pose: PoseStamped, speed: float = None):
        """Send goal to move_base ActionServer."""
        if speed is not None:
            self._set_dwa_speed(speed)

        mb_goal = MoveBaseGoal()
        mb_goal.target_pose = goal_pose
        mb_goal.target_pose.header.frame_id = 'map'
        mb_goal.target_pose.header.stamp    = rospy.Time.now()

        self.mb_client.send_goal(
            mb_goal,
            done_cb=self._nav_done_cb,
            active_cb=None,
            feedback_cb=None
        )
        self.goal_sent_time = time.time()
        self.nav_active = True
        rospy.loginfo("[FSM] Nav goal sent: (%.3f, %.3f) speed=%.2fm/s",
                      goal_pose.pose.position.x,
                      goal_pose.pose.position.y,
                      speed if speed else self.delivery_speed)

    def _cancel_nav(self):
        """Cancel all move_base goals."""
        self.mb_client.cancel_all_goals()
        self.nav_active = False

    def _nav_done_cb(self, status, result):
        """move_base completion callback."""
        self.nav_active = False
        status_str = {
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED:   "ABORTED (blocked)",
            GoalStatus.PREEMPTED: "PREEMPTED",
        }.get(status, f"STATUS_{status}")
        rospy.loginfo("[FSM] move_base done: %s", status_str)

    def _set_dwa_speed(self, speed: float):
        """Adjust DWA max velocity via dynamic_reconfigure."""
        speed = max(0.02, min(speed, 0.22))
        if self.dwa_client is not None:
            try:
                self.dwa_client.update_configuration({
                    'max_vel_x':     speed,
                    'max_vel_trans': speed
                })
            except Exception as e:
                rospy.logwarn_throttle(
                    10, "[FSM] DWA reconfigure failed: %s", str(e))

    def _stop_robot(self):
        """Publish zero-velocity to /cmd_vel."""
        self.cmd_vel_pub.publish(Twist())

    # =========================================================================
    #  GOAL REACHED CHECK
    # =========================================================================

    def _is_goal_reached(self) -> bool:
        """
        Returns True when robot is within goal_tolerance_xy AND goal_tolerance_yaw
        of current_goal.
        Position: ≤ 0.10m | Yaw: ≤ 5° (as per spec)
        """
        if self.current_pose is None or self.current_goal is None:
            return False

        dist = self._distance_to_goal()
        if dist > self.goal_tolerance_xy:
            return False

        # Yaw comparison
        q_c = self.current_pose.orientation
        q_g = self.current_goal.pose.orientation
        _, _, yaw_curr = tf.transformations.euler_from_quaternion(
            [q_c.x, q_c.y, q_c.z, q_c.w])
        _, _, yaw_goal = tf.transformations.euler_from_quaternion(
            [q_g.x, q_g.y, q_g.z, q_g.w])

        yaw_err = abs(self._wrap_angle(yaw_curr - yaw_goal))
        return yaw_err <= self.goal_tolerance_yaw_rad

    def _distance_to_goal(self) -> float:
        """Euclidean distance from current AMCL pose to active goal."""
        if self.current_pose is None or self.current_goal is None:
            return float('inf')
        dx = self.current_pose.position.x - self.current_goal.pose.position.x
        dy = self.current_pose.position.y - self.current_goal.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    # =========================================================================
    #  GOAL PARSER
    # =========================================================================

    def _parse_goal(self, cmd: str):
        """
        Parse task string into PoseStamped.
        Accepts: "table_1" | "x,y" | "x,y,yaw_deg"
        Returns: PoseStamped or None
        """
        # Named location
        if cmd in self.location_map:
            coords = self.location_map[cmd]
            x   = float(coords[0])
            y   = float(coords[1])
            yaw = float(coords[2]) if len(coords) > 2 else 0.0
            return self._make_pose_stamped(x, y, yaw)

        # Raw coordinates
        parts = [p.strip() for p in cmd.split(',')]
        if len(parts) in (2, 3):
            try:
                x   = float(parts[0])
                y   = float(parts[1])
                yaw = float(parts[2]) if len(parts) == 3 else 0.0
                return self._make_pose_stamped(x, y, yaw)
            except ValueError:
                pass

        return None

    @staticmethod
    def _make_pose_stamped(x: float, y: float, yaw_deg: float) -> PoseStamped:
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

    # ── Spin ──────────────────────────────────────────────────────────────────
    def spin(self):
        rospy.spin()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    try:
        fsm = FoodDeliveryFSM()
        fsm.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[FSM] Shutdown requested.")


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
navigation_control.py — move_base Navigation Interface
=======================================================
Manages all interaction with the move_base action server.
Provides:
  - send_goal()       : Send nav goal to move_base
  - cancel_goal()     : Cancel current navigation (for obstacle avoidance)
  - resume_goal()     : Resend saved goal after avoidance
  - set_speed()       : Adjust max velocity via dynamic_reconfigure
  - is_goal_reached() : Check arrival within tolerance

Subscribes:  /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)
             /nav_goal  (geometry_msgs/PoseStamped) — from task_manager
Publishes:   /cmd_vel   (geometry_msgs/Twist)       — emergency stop
Actions:     move_base  (move_base_msgs/MoveBaseAction)

Run standalone:
  rosrun turtlebot3_delivery navigation_control.py

Dependencies:
  sudo apt install ros-noetic-move-base-msgs ros-noetic-dynamic-reconfigure
"""

import math
import threading
import rospy
import actionlib
import tf

from geometry_msgs.msg import (
    PoseStamped, PoseWithCovarianceStamped, Twist, Point, Quaternion
)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from dynamic_reconfigure.client import Client as DynClient


class NavigationControl:
    """
    Wraps move_base ActionLib client with speed control and
    goal state tracking for FSM integration.
    """

    # move_base goal status codes
    STATUS_PENDING   = GoalStatus.PENDING
    STATUS_ACTIVE    = GoalStatus.ACTIVE
    STATUS_SUCCEEDED = GoalStatus.SUCCEEDED
    STATUS_ABORTED   = GoalStatus.ABORTED
    STATUS_PREEMPTED = GoalStatus.PREEMPTED

    def __init__(self):
        rospy.init_node('navigation_control', anonymous=False)

        # ── Parameters ──────────────────────────────────────────────────────
        self.delivery_speed        = rospy.get_param('~delivery_speed',        0.15)
        self.avoidance_speed       = rospy.get_param('~avoidance_speed',       0.05)
        self.goal_tolerance_xy     = rospy.get_param('~goal_tolerance_xy',     0.10)
        self.goal_tolerance_yaw_deg = rospy.get_param('~goal_tolerance_yaw_deg', 5.0)
        self.nav_goal_timeout      = rospy.get_param('~nav_goal_timeout',     120.0)

        self.goal_tolerance_yaw_rad = math.radians(self.goal_tolerance_yaw_deg)

        # ── State ────────────────────────────────────────────────────────────
        self.current_pose    = None       # Latest AMCL pose
        self.current_goal    = None       # Active PoseStamped goal
        self.goal_lock       = threading.Lock()
        self.nav_active      = False
        self.last_goal_status = None

        # ── move_base Action Client ──────────────────────────────────────────
        self.move_base_client = actionlib.SimpleActionClient(
            'move_base', MoveBaseAction)
        rospy.loginfo("[NavControl] Waiting for move_base action server...")
        connected = self.move_base_client.wait_for_server(
            timeout=rospy.Duration(30.0))
        if not connected:
            rospy.logerr("[NavControl] move_base server not found after 30s! "
                         "Is navigation.launch running?")
        else:
            rospy.loginfo("[NavControl] move_base action server connected.")

        # ── dynamic_reconfigure for speed adjustment ─────────────────────────
        try:
            self.dwa_client = DynClient(
                '/move_base/DWAPlannerROS',
                timeout=5.0
            )
            rospy.loginfo("[NavControl] DWA dynamic_reconfigure client connected.")
        except Exception as e:
            self.dwa_client = None
            rospy.logwarn("[NavControl] DWA reconfigure unavailable: %s. "
                          "Speed changes will use cmd_vel only.", str(e))

        # ── Subscribers ──────────────────────────────────────────────────────
        self.pose_sub = rospy.Subscriber(
            '/amcl_pose',
            PoseWithCovarianceStamped,
            self._amcl_pose_callback,
            queue_size=5
        )

        self.nav_goal_sub = rospy.Subscriber(
            '/nav_goal',
            PoseStamped,
            self._nav_goal_callback,
            queue_size=5
        )

        # ── Publishers ───────────────────────────────────────────────────────
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=5)

        # ── TF listener for goal-reached check ──────────────────────────────
        self.tf_listener = tf.TransformListener()

        rospy.loginfo("[NavControl] Initialized. Delivery speed=%.2fm/s, "
                      "Avoidance speed=%.2fm/s, Goal tolerance=%.2fm",
                      self.delivery_speed, self.avoidance_speed,
                      self.goal_tolerance_xy)

    # ── Subscriber callbacks ─────────────────────────────────────────────────
    def _amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Store latest AMCL localization pose."""
        self.current_pose = msg.pose.pose

    def _nav_goal_callback(self, msg: PoseStamped):
        """
        External goal received (from task_manager).
        Auto-sends to move_base at delivery speed.
        """
        rospy.loginfo("[NavControl] Received /nav_goal: (%.2f, %.2f)",
                      msg.pose.position.x, msg.pose.position.y)
        self.send_goal(msg, speed=self.delivery_speed)

    # ── Core navigation API ──────────────────────────────────────────────────
    def send_goal(self, goal_pose: PoseStamped, speed: float = None) -> bool:
        """
        Send a navigation goal to move_base.
        
        Args:
            goal_pose: Target pose in map frame
            speed:     Override linear speed (m/s). None = use delivery_speed
        Returns:
            True if goal was accepted
        """
        if speed is None:
            speed = self.delivery_speed

        with self.goal_lock:
            self.current_goal = goal_pose

        # Adjust planner speed before sending goal
        self._set_max_speed(speed)

        # Build MoveBaseGoal
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose = goal_pose
        mb_goal.target_pose.header.frame_id = 'map'
        mb_goal.target_pose.header.stamp    = rospy.Time.now()

        self.move_base_client.send_goal(
            mb_goal,
            done_cb=self._goal_done_callback,
            active_cb=self._goal_active_callback,
            feedback_cb=self._goal_feedback_callback
        )
        self.nav_active = True
        rospy.loginfo("[NavControl] Goal sent: (%.3f, %.3f) at speed=%.2fm/s",
                      goal_pose.pose.position.x,
                      goal_pose.pose.position.y,
                      speed)
        return True

    def cancel_goal(self):
        """
        Cancel current move_base goal.
        Called when FSM transitions Delivering → AvoidingObstacle.
        Publishes zero velocity to immediately stop the robot.
        """
        self.move_base_client.cancel_all_goals()
        self.nav_active = False
        self.stop_robot()
        rospy.loginfo("[NavControl] Goal cancelled. Robot stopped.")

    def resume_goal(self) -> bool:
        """
        Resend the saved delivery goal after obstacle avoidance.
        Called when FSM transitions AvoidingObstacle → DeliveringFood.
        Restores delivery speed.
        """
        with self.goal_lock:
            goal = self.current_goal

        if goal is None:
            rospy.logerr("[NavControl] resume_goal() called but no saved goal!")
            return False

        rospy.loginfo("[NavControl] Resuming delivery to saved goal "
                      "(%.3f, %.3f)", goal.pose.position.x, goal.pose.position.y)
        return self.send_goal(goal, speed=self.delivery_speed)

    def stop_robot(self):
        """Publish zero velocity — immediate full stop."""
        self.cmd_vel_pub.publish(Twist())

    # ── Speed control ────────────────────────────────────────────────────────
    def _set_max_speed(self, speed: float):
        """
        Adjust DWA planner max velocity via dynamic_reconfigure.
        Falls back to capping via cost if DWA client unavailable.
        """
        speed = max(0.02, min(speed, 0.22))   # Clamp to Waffle hardware limits
        if self.dwa_client is not None:
            try:
                self.dwa_client.update_configuration({
                    'max_vel_x':     speed,
                    'max_vel_trans': speed
                })
                rospy.logdebug("[NavControl] DWA max_vel_x set to %.2f", speed)
            except Exception as e:
                rospy.logwarn_throttle(
                    10, "[NavControl] DWA reconfigure failed: %s", str(e))

    def set_avoidance_speed(self):
        """Switch to slow avoidance speed (0.05 m/s)."""
        self._set_max_speed(self.avoidance_speed)
        rospy.loginfo("[NavControl] Speed → AVOIDANCE (%.2fm/s)",
                      self.avoidance_speed)

    def set_delivery_speed(self):
        """Restore normal delivery speed (0.15 m/s)."""
        self._set_max_speed(self.delivery_speed)
        rospy.loginfo("[NavControl] Speed → DELIVERY (%.2fm/s)",
                      self.delivery_speed)

    # ── Goal arrival check ───────────────────────────────────────────────────
    def is_goal_reached(self) -> bool:
        """
        Check if robot is within tolerance of current goal.
        Uses AMCL pose vs goal pose.
        Position: ≤ 0.10m (x/y) | Yaw: ≤ 5°
        """
        if self.current_pose is None or self.current_goal is None:
            return False

        # ── Position check ──────────────────────────────────────────────────
        dx = self.current_pose.position.x - self.current_goal.pose.position.x
        dy = self.current_pose.position.y - self.current_goal.pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist > self.goal_tolerance_xy:
            return False

        # ── Yaw check ───────────────────────────────────────────────────────
        # Extract current yaw from AMCL quaternion
        q_curr = self.current_pose.orientation
        _, _, yaw_curr = tf.transformations.euler_from_quaternion(
            [q_curr.x, q_curr.y, q_curr.z, q_curr.w])

        # Extract goal yaw from goal quaternion
        q_goal = self.current_goal.pose.orientation
        _, _, yaw_goal = tf.transformations.euler_from_quaternion(
            [q_goal.x, q_goal.y, q_goal.z, q_goal.w])

        yaw_err = abs(self._normalize_angle(yaw_curr - yaw_goal))
        return yaw_err <= self.goal_tolerance_yaw_rad

    def get_distance_to_goal(self) -> float:
        """Returns current Euclidean distance to the active goal (m)."""
        if self.current_pose is None or self.current_goal is None:
            return float('inf')
        dx = self.current_pose.position.x - self.current_goal.pose.position.x
        dy = self.current_pose.position.y - self.current_goal.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    # ── Action callbacks ─────────────────────────────────────────────────────
    def _goal_done_callback(self, status, result):
        self.nav_active = False
        status_map = {
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED:   "ABORTED",
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.REJECTED:  "REJECTED",
        }
        self.last_goal_status = status
        rospy.loginfo("[NavControl] Goal done: %s",
                      status_map.get(status, f"CODE_{status}"))

    def _goal_active_callback(self):
        rospy.logdebug("[NavControl] Goal is now active.")

    def _goal_feedback_callback(self, feedback):
        rospy.logdebug_throttle(
            5, "[NavControl] Feedback: base_position=(%.2f, %.2f)",
            feedback.base_position.pose.position.x,
            feedback.base_position.pose.position.y
        )

    # ── Utility ──────────────────────────────────────────────────────────────
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-π, π]."""
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    @staticmethod
    def make_pose_stamped(x: float, y: float,
                           yaw_deg: float = 0.0) -> PoseStamped:
        """
        Convenience factory: create PoseStamped from x, y, yaw (degrees).
        Returns pose in 'map' frame.
        """
        q = tf.transformations.quaternion_from_euler(0, 0, math.radians(yaw_deg))
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = rospy.Time.now()
        pose.pose.position   = Point(x=x, y=y, z=0.0)
        pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def spin(self):
        rospy.spin()


# ── Entry point ──────────────────────────────────────────────────────────────
def main():
    try:
        ctrl = NavigationControl()
        ctrl.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[NavControl] Shutting down.")


if __name__ == '__main__':
    main()

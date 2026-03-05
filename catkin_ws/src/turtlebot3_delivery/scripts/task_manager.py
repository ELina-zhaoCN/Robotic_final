#!/usr/bin/env python3
"""
task_manager.py — Delivery Task Queue & Goal Manager
=====================================================
Receives delivery task commands, resolves location names to map coordinates,
and publishes navigation goals to /nav_goal.

Subscribes:  /delivery_task  (std_msgs/String)  — e.g. "table_1" or "3.5,2.0,90"
             /finish_task    (std_msgs/Bool)     — task completion signal
             /robot_state    (turtlebot3_delivery/RobotState)

Publishes:   /nav_goal       (geometry_msgs/PoseStamped) — to navigation_control
             /finish_task    (std_msgs/Bool)              — republish on completion

Run standalone:
  rosrun turtlebot3_delivery task_manager.py

Send a task (by location name):
  rostopic pub /delivery_task std_msgs/String "data: 'table_1'"

Send a task (by raw coordinates: x,y,yaw_degrees):
  rostopic pub /delivery_task std_msgs/String "data: '3.5,2.0,90'"
"""

import math
import threading
from collections import deque

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from std_msgs.msg import String, Bool


class TaskManager:
    """
    Manages a FIFO queue of delivery tasks.
    Only one task executes at a time; subsequent tasks queue up.
    """

    def __init__(self):
        rospy.init_node('task_manager', anonymous=False)

        # ── Location database (loaded from fsm_params.yaml) ─────────────────
        # Format: location_name → [x, y, yaw_degrees]
        self.location_map = self._load_locations()

        # ── Task queue ────────────────────────────────────────────────────────
        self.task_queue   = deque()
        self.queue_lock   = threading.Lock()
        self.current_task = None      # Currently executing task dict
        self.robot_idle   = True      # True when FSM is in Idle state

        # ── Subscribers ──────────────────────────────────────────────────────
        self.task_sub = rospy.Subscriber(
            '/delivery_task', String,
            self._task_callback,
            queue_size=10
        )
        self.finish_sub = rospy.Subscriber(
            '/finish_task', Bool,
            self._finish_callback,
            queue_size=5
        )
        # Monitor FSM state to know when robot returns to Idle
        self.state_sub = rospy.Subscriber(
            '/robot_state', String,
            self._state_callback,
            queue_size=5
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.nav_goal_pub   = rospy.Publisher(
            '/nav_goal', PoseStamped, queue_size=5)
        self.finish_task_pub = rospy.Publisher(
            '/finish_task', Bool, queue_size=5)
        self.status_pub     = rospy.Publisher(
            '/task_status', String, queue_size=5)

        # ── Queue drain timer (checks for pending tasks every 0.5s) ─────────
        self.drain_timer = rospy.Timer(
            rospy.Duration(0.5), self._drain_queue)

        self._log_location_map()
        rospy.loginfo("[TaskManager] Ready. Send tasks via: "
                      "rostopic pub /delivery_task std_msgs/String "
                      "\"data: 'table_1'\"")

    # ── Location database loader ─────────────────────────────────────────────
    def _load_locations(self) -> dict:
        """Load pre-defined locations from ROS params (fsm_params.yaml)."""
        locations = {}

        param_locs = rospy.get_param('/fsm/locations', {})
        for name, coords in param_locs.items():
            if len(coords) >= 2:
                locations[name] = {
                    'x':   float(coords[0]),
                    'y':   float(coords[1]),
                    'yaw': float(coords[2]) if len(coords) > 2 else 0.0
                }

        # Always define kitchen as home position
        if 'kitchen' not in locations:
            locations['kitchen'] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}

        return locations

    def _log_location_map(self):
        rospy.loginfo("[TaskManager] Loaded %d delivery locations:",
                      len(self.location_map))
        for name, loc in self.location_map.items():
            rospy.loginfo("  %-12s → (%.2f, %.2f, yaw=%.1f°)",
                          name, loc['x'], loc['y'], loc['yaw'])

    # ── Subscriber callbacks ─────────────────────────────────────────────────
    def _task_callback(self, msg: String):
        """
        Parse incoming task command.
        Accepts:
          "table_1"           — named location from location_map
          "3.5,2.0"           — raw x,y coordinates (yaw=0)
          "3.5,2.0,90"        — raw x,y,yaw_degrees
        """
        cmd = msg.data.strip()
        rospy.loginfo("[TaskManager] Received task command: '%s'", cmd)

        task = self._parse_task(cmd)
        if task is None:
            rospy.logerr(
                "[TaskManager] Cannot parse task '%s'. "
                "Use location name or 'x,y' or 'x,y,yaw'.", cmd)
            return

        with self.queue_lock:
            self.task_queue.append(task)
            rospy.loginfo(
                "[TaskManager] Task queued: %s → (%.2f, %.2f, %.1f°). "
                "Queue size: %d",
                task['name'], task['x'], task['y'], task['yaw'],
                len(self.task_queue)
            )

    def _finish_callback(self, msg: Bool):
        """Called when FSM signals task completion."""
        if msg.data:
            rospy.loginfo("[TaskManager] Task completed signal received.")
            self.current_task = None
            self.robot_idle   = True
            self.status_pub.publish(String(data="TASK_COMPLETED"))

    def _state_callback(self, msg: String):
        """Monitor FSM state string — update idle flag."""
        self.robot_idle = (msg.data == "IDLE")

    # ── Queue drain ───────────────────────────────────────────────────────────
    def _drain_queue(self, event):
        """
        Every 0.5s: if robot is idle and queue has tasks, dispatch next task.
        """
        with self.queue_lock:
            if not self.task_queue:
                return
            if not self.robot_idle:
                return
            if self.current_task is not None:
                return

            task = self.task_queue.popleft()

        self._dispatch_task(task)

    def _dispatch_task(self, task: dict):
        """Send goal pose to navigation_control via /nav_goal."""
        self.current_task = task
        self.robot_idle   = False

        goal = self._make_pose_stamped(task['x'], task['y'], task['yaw'])
        self.nav_goal_pub.publish(goal)

        self.status_pub.publish(
            String(data=f"DISPATCHED:{task['name']}"))

        rospy.loginfo(
            "[TaskManager] Dispatched: %s → (%.2f, %.2f, yaw=%.1f°)",
            task['name'], task['x'], task['y'], task['yaw'])

    # ── Task parsing ──────────────────────────────────────────────────────────
    def _parse_task(self, cmd: str) -> dict:
        """
        Returns task dict: {name, x, y, yaw} or None on parse error.
        """
        # ── Try named location first ─────────────────────────────────────────
        if cmd in self.location_map:
            loc = self.location_map[cmd]
            return {'name': cmd,
                    'x': loc['x'], 'y': loc['y'], 'yaw': loc['yaw']}

        # ── Try coordinate format: "x,y" or "x,y,yaw" ──────────────────────
        parts = [p.strip() for p in cmd.split(',')]
        if len(parts) in (2, 3):
            try:
                x   = float(parts[0])
                y   = float(parts[1])
                yaw = float(parts[2]) if len(parts) == 3 else 0.0
                return {'name': f"coord({x:.2f},{y:.2f})",
                        'x': x, 'y': y, 'yaw': yaw}
            except ValueError:
                pass

        return None

    # ── Helpers ──────────────────────────────────────────────────────────────
    @staticmethod
    def _make_pose_stamped(x: float, y: float,
                            yaw_deg: float = 0.0) -> PoseStamped:
        q = tf.transformations.quaternion_from_euler(
            0, 0, math.radians(yaw_deg))
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp    = rospy.Time.now()
        pose.pose.position   = Point(x=x, y=y, z=0.0)
        pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3])
        return pose

    def spin(self):
        rospy.spin()


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    try:
        manager = TaskManager()
        manager.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("[TaskManager] Shutting down.")


if __name__ == '__main__':
    main()

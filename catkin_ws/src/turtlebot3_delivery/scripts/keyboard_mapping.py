#!/usr/bin/env python3
"""
keyboard_mapping.py — Keyboard-Controlled Robot Driving for SLAM Mapping
=========================================================================
Drive the TurtleBot3 Waffle with your keyboard while gmapping
automatically builds the restaurant map in the background.

Controls:
  W / ↑   — Move forward
  S / ↓   — Move backward
  A / ←   — Turn left
  D / →   — Turn right
  SPACE   — Emergency stop (zero velocity)
  Q       — Quit and save map

Speed adjustment:
  + / =   — Increase linear speed
  - / _   — Decrease linear speed

Publishes: /cmd_vel (geometry_msgs/Twist)

Run:
  # Terminal 1: Start mapping stack
  roslaunch turtlebot3_delivery mapping.launch
  
  # Terminal 2: Drive the robot
  python3 scripts/keyboard_mapping.py

Dependencies:
  pip3 install pynput
"""

import sys
import os
import threading
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid

# Use pynput for non-blocking keyboard input (no terminal raw mode needed)
try:
    from pynput import keyboard as pynput_kb
    PYNPUT_OK = True
except ImportError:
    PYNPUT_OK = False
    print("[KeyMapping] pynput not found. Installing...")
    os.system("pip3 install pynput")
    print("[KeyMapping] Please re-run the script.")
    sys.exit(1)


# ── Speed constants (TurtleBot3 Waffle hardware limits) ─────────────────────
LINEAR_SPEED_DEFAULT  = 0.15   # m/s — safe mapping speed
LINEAR_SPEED_MIN      = 0.05   # m/s — slowest
LINEAR_SPEED_MAX      = 0.22   # m/s — Waffle hardware max
LINEAR_SPEED_STEP     = 0.02   # Speed increment per key press

ANGULAR_SPEED_DEFAULT = 0.60   # rad/s — ~35°/s, smooth turning
ANGULAR_SPEED_MAX     = 1.00   # rad/s
ANGULAR_SPEED_STEP    = 0.10


class KeyboardMapper:
    """
    Reads keyboard input and publishes velocity commands to /cmd_vel.
    Designed for use during gmapping SLAM sessions.
    """

    CONTROLS = """
╔════════════════════════════════════════╗
║   Keyboard Mapping Controls           ║
╠════════════════════════════════════════╣
║  W / ↑        Forward                 ║
║  S / ↓        Backward                ║
║  A / ←        Turn Left               ║
║  D / →        Turn Right              ║
║  SPACE        STOP (zero velocity)    ║
║  + / =        Increase speed          ║
║  - / _        Decrease speed          ║
║  M            Save map now            ║
║  Q / ESC      Quit                    ║
╚════════════════════════════════════════╝
"""

    def __init__(self):
        rospy.init_node('keyboard_mapper', anonymous=False)

        # ── Speed state ──────────────────────────────────────────────────
        self.linear_speed  = LINEAR_SPEED_DEFAULT
        self.angular_speed = ANGULAR_SPEED_DEFAULT

        # ── Key state (which keys are currently held down) ────────────────
        self.keys_held = set()
        self.running   = True

        # ── Map quality monitoring ────────────────────────────────────────
        self.map_cell_count  = 0
        self.map_update_time = None

        # ── Publisher ─────────────────────────────────────────────────────
        self.cmd_vel_pub = rospy.Publisher(
            '/cmd_vel', Twist, queue_size=5)

        # ── Map subscriber (for live quality feedback) ────────────────────
        rospy.Subscriber('/map', OccupancyGrid,
                         self._map_callback, queue_size=1)

        # ── Velocity publish timer (20Hz for smooth motion) ───────────────
        self.pub_timer = rospy.Timer(
            rospy.Duration(0.05),   # 20Hz
            self._publish_velocity
        )

        # ── Status display timer (1Hz) ────────────────────────────────────
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0),
            self._print_status
        )

        print(self.CONTROLS)
        print(f"[KeyMapping] Linear speed:  {self.linear_speed:.2f} m/s")
        print(f"[KeyMapping] Angular speed: {self.angular_speed:.2f} rad/s")
        print("[KeyMapping] Drive the robot to cover the entire restaurant.")
        print("[KeyMapping] Watch RViz — make sure all walls are visible.\n")

    # ── Map quality callback ─────────────────────────────────────────────────
    def _map_callback(self, msg: OccupancyGrid):
        """Count known (non-unknown) cells as a map quality metric."""
        known = sum(1 for c in msg.data if c >= 0)   # -1 = unknown
        self.map_cell_count  = known
        self.map_update_time = time.time()

    # ── Velocity computation from held keys ──────────────────────────────────
    def _compute_twist(self) -> Twist:
        """
        Convert currently held keys to a Twist velocity command.
        Diagonal keys (e.g., W+A) create arc motion.
        """
        twist = Twist()

        # Forward / Backward
        if pynput_kb.Key.up   in self.keys_held or 'w' in self.keys_held:
            twist.linear.x =  self.linear_speed
        if pynput_kb.Key.down in self.keys_held or 's' in self.keys_held:
            twist.linear.x = -self.linear_speed

        # Left / Right rotation
        if pynput_kb.Key.left  in self.keys_held or 'a' in self.keys_held:
            twist.angular.z =  self.angular_speed
        if pynput_kb.Key.right in self.keys_held or 'd' in self.keys_held:
            twist.angular.z = -self.angular_speed

        return twist

    # ── Velocity publish callback ─────────────────────────────────────────────
    def _publish_velocity(self, event):
        """Publish velocity at 20Hz while keys are held."""
        if not self.running:
            return
        twist = self._compute_twist()
        self.cmd_vel_pub.publish(twist)

    # ── Status display ────────────────────────────────────────────────────────
    def _print_status(self, event):
        """Print live mapping status once per second."""
        age = ""
        if self.map_update_time:
            secs = time.time() - self.map_update_time
            age  = f"  Map updated {secs:.0f}s ago"

        cells_m2 = self.map_cell_count * 0.05 * 0.05   # cells × resolution²
        print(f"\r[KeyMapping] Speed: {self.linear_speed:.2f}m/s | "
              f"Map: {self.map_cell_count} cells ({cells_m2:.1f}m²){age}    ",
              end="", flush=True)

    # ── Keyboard event handlers ───────────────────────────────────────────────
    def _on_press(self, key):
        """Handle key press — add to held set or execute one-shot action."""
        self.keys_held.add(key)

        try:
            ch = key.char.lower() if hasattr(key, 'char') and key.char else None
        except AttributeError:
            ch = None

        # ── One-shot actions ──────────────────────────────────────────────
        if key == pynput_kb.Key.space:
            # Emergency stop
            self.keys_held.clear()
            self.cmd_vel_pub.publish(Twist())
            print("\n[KeyMapping] STOPPED (SPACE)")

        elif ch == '+' or ch == '=':
            self.linear_speed = min(self.linear_speed + LINEAR_SPEED_STEP,
                                    LINEAR_SPEED_MAX)
            print(f"\n[KeyMapping] Speed: {self.linear_speed:.2f} m/s")

        elif ch == '-' or ch == '_':
            self.linear_speed = max(self.linear_speed - LINEAR_SPEED_STEP,
                                    LINEAR_SPEED_MIN)
            print(f"\n[KeyMapping] Speed: {self.linear_speed:.2f} m/s")

        elif ch == 'm':
            print("\n[KeyMapping] Saving map...")
            self._save_map_now()

        elif ch == 'q' or key == pynput_kb.Key.esc:
            print("\n[KeyMapping] Quit requested. Stopping robot...")
            self.cmd_vel_pub.publish(Twist())
            self.running = False
            return False   # Stop pynput listener

    def _on_release(self, key):
        """Handle key release — remove from held set."""
        try:
            self.keys_held.discard(key)
        except Exception:
            pass

        # If no movement keys held → publish zero velocity once
        movement_keys = {
            pynput_kb.Key.up, pynput_kb.Key.down,
            pynput_kb.Key.left, pynput_kb.Key.right
        }
        char_keys = {'w', 's', 'a', 'd'}

        held_chars = {
            k.char.lower() for k in self.keys_held
            if hasattr(k, 'char') and k.char
        }

        if not (movement_keys & self.keys_held) and not (char_keys & held_chars):
            self.cmd_vel_pub.publish(Twist())

    # ── Map save ──────────────────────────────────────────────────────────────
    def _save_map_now(self):
        """Trigger map_tools.py save from within this script."""
        import subprocess
        scripts_dir = os.path.dirname(os.path.abspath(__file__))
        map_tools   = os.path.join(scripts_dir, "map_tools.py")
        try:
            result = subprocess.run(
                ["python3", map_tools, "save"],
                timeout=20, capture_output=True, text=True
            )
            if result.returncode == 0:
                print("[KeyMapping] Map saved successfully!")
            else:
                print(f"[KeyMapping] Map save failed: {result.stderr[:200]}")
        except Exception as e:
            print(f"[KeyMapping] Save error: {e}")

    # ── Main run ───────────────────────────────────────────────────────────────
    def run(self):
        """Start keyboard listener and block until quit."""
        with pynput_kb.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        ) as listener:
            # Wait for ROS or quit signal
            while self.running and not rospy.is_shutdown():
                time.sleep(0.1)
            listener.stop()

        # Final stop command
        self.cmd_vel_pub.publish(Twist())
        print("\n[KeyMapping] Robot stopped.")
        print("\n[KeyMapping] IMPORTANT: Save the map before quitting!")
        print("  python3 scripts/map_tools.py save")
        print("  OR: Map was auto-saved if you pressed 'M'\n")


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    try:
        mapper = KeyboardMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        print("\n[KeyMapping] ROS shutdown.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
mark_goal_points.py — Interactive Restaurant Goal Point Annotator
=================================================================
After building the map, use this script to mark delivery locations
(kitchen, table_1, table_2, ...) by clicking on the map in RViz.

Workflow:
  1. Run navigation.launch (map + AMCL + move_base)
  2. Run this script in a new terminal
  3. In RViz, add "ClickedPoint" display (or use "Publish Point" tool)
  4. Click on each location on the map
  5. Type the location name when prompted
  6. Script saves coordinates to fsm_map_config.yaml automatically

Alternatively (no RViz): Enter coordinates manually via text input.

Subscribes:  /clicked_point (geometry_msgs/PointStamped) — RViz click
             /amcl_pose     (geometry_msgs/PoseWithCovarianceStamped)

Publishes:   /marked_goals  (visualization_msgs/MarkerArray) — RViz markers
             /goal_markers  (visualization_msgs/Marker)      — individual

Run:
  # Make sure navigation.launch is running first
  roslaunch turtlebot3_delivery navigation.launch map_file:=~/maps/restaurant_map.yaml
  
  # Then in a new terminal:
  python3 scripts/mark_goal_points.py

Dependencies:
  sudo apt install ros-noetic-visualization-msgs
"""

import os
import math
import yaml
import time
import threading
from datetime import datetime

import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped, Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, ColorRGBA


# ── Configuration ────────────────────────────────────────────────────────────
PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_FILE = os.path.join(PACKAGE_DIR, "config", "fsm_map_config.yaml")

# Predefined location slots to fill (shown as prompts to the user)
LOCATION_SLOTS = [
    "kitchen",
    "table_1",
    "table_2",
    "table_3",
    "table_4",
    "table_5",
]

# Marker colors per location type
MARKER_COLORS = {
    "kitchen": (1.0, 0.5, 0.0),    # Orange
    "table_1": (0.0, 0.8, 0.0),    # Green
    "table_2": (0.0, 0.8, 0.0),
    "table_3": (0.0, 0.8, 0.0),
    "table_4": (0.0, 0.8, 0.0),
    "table_5": (0.0, 0.8, 0.0),
}
DEFAULT_COLOR = (0.0, 0.5, 1.0)    # Blue for custom locations


class GoalPointMarker:
    """
    Interactive tool for annotating delivery locations on the restaurant map.
    Supports both RViz click input and manual coordinate entry.
    """

    def __init__(self):
        rospy.init_node('mark_goal_points', anonymous=False)

        # ── Stored locations ──────────────────────────────────────────────
        self.locations   = {}      # name → {'x': float, 'y': float, 'yaw': float}
        self.pending_name = None   # Location name waiting for a click
        self.lock = threading.Lock()

        # ── Load existing locations from config ──────────────────────────
        self._load_existing_locations()

        # ── TF listener (for yaw extraction from poses) ──────────────────
        self.tf_listener = tf.TransformListener()

        # ── Subscribers ──────────────────────────────────────────────────
        rospy.Subscriber('/clicked_point', PointStamped,
                         self._click_callback, queue_size=5)

        # ── Publishers ───────────────────────────────────────────────────
        self.marker_pub  = rospy.Publisher(
            '/goal_markers', MarkerArray, queue_size=5, latch=True)

        # Re-publish existing markers every 5s so they stay visible in RViz
        self.vis_timer = rospy.Timer(
            rospy.Duration(5.0), self._republish_markers)

        rospy.loginfo("[MarkGoals] Node started. Config: %s", CONFIG_FILE)

    # ── Load existing locations ───────────────────────────────────────────────
    def _load_existing_locations(self):
        """Load previously saved locations from fsm_map_config.yaml."""
        if not os.path.exists(CONFIG_FILE):
            rospy.logwarn("[MarkGoals] Config file not found: %s", CONFIG_FILE)
            return

        try:
            with open(CONFIG_FILE, 'r') as f:
                config = yaml.safe_load(f)
            locs = config.get('locations', {})
            for name, coords in locs.items():
                if len(coords) >= 2:
                    self.locations[name] = {
                        'x':   float(coords[0]),
                        'y':   float(coords[1]),
                        'yaw': float(coords[2]) if len(coords) > 2 else 0.0
                    }
            rospy.loginfo("[MarkGoals] Loaded %d existing locations.",
                          len(self.locations))
        except Exception as e:
            rospy.logwarn("[MarkGoals] Could not load config: %s", str(e))

    # ── RViz click callback ───────────────────────────────────────────────────
    def _click_callback(self, msg: PointStamped):
        """
        Called when user clicks 'Publish Point' in RViz.
        Assigns the clicked coordinate to self.pending_name.
        """
        x = msg.point.x
        y = msg.point.y

        with self.lock:
            name = self.pending_name

        if name is None:
            rospy.logwarn(
                "[MarkGoals] Point clicked (%.3f, %.3f) but no location name "
                "is pending. Run the interactive prompt first.", x, y)
            return

        rospy.loginfo("[MarkGoals] Point received for '%s': (%.3f, %.3f)",
                      name, x, y)

        with self.lock:
            self.locations[name] = {'x': x, 'y': y, 'yaw': 0.0}
            self.pending_name = None

        print(f"\n[MarkGoals] ✓ '{name}' saved at ({x:.3f}, {y:.3f})")
        print("[MarkGoals] Tip: Press Enter to set the yaw (facing direction),")
        print("            or just Enter to keep yaw=0 (facing +x axis).\n")

        # Publish updated markers
        self._republish_markers(None)

    # ── Interactive annotation session ────────────────────────────────────────
    def run_interactive_session(self):
        """
        Main interactive loop. Supports both click-based and manual input.
        """
        print("\n" + "=" * 60)
        print("  Restaurant Goal Point Annotator")
        print("=" * 60)
        print("Make sure RViz is open with the map visible.")
        print("In RViz: Click the 'Publish Point' button (crosshair icon)")
        print("         then click on the map to set a location.\n")
        print("Already marked locations:")
        for name, loc in self.locations.items():
            print(f"  {name:12s} → ({loc['x']:.2f}, {loc['y']:.2f}, "
                  f"yaw={loc['yaw']:.1f}°)")
        print()

        # Offer choices
        print("Options:")
        print("  1. Mark pre-defined locations (kitchen + table_1~5)")
        print("  2. Mark a custom location")
        print("  3. Enter coordinates manually (no RViz needed)")
        print("  4. Save and quit")
        print()

        while not rospy.is_shutdown():
            try:
                choice = input("Select option (1/2/3/4): ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if choice == '1':
                self._mark_predefined_locations()
            elif choice == '2':
                self._mark_custom_location()
            elif choice == '3':
                self._manual_entry()
            elif choice == '4':
                break
            else:
                print("Invalid choice. Enter 1, 2, 3, or 4.")

        self._save_to_config()
        print("\n[MarkGoals] All done! Locations saved to:")
        print(f"  {CONFIG_FILE}")
        self._print_summary()

    # ── Mark pre-defined locations ────────────────────────────────────────────
    def _mark_predefined_locations(self):
        """Walk through each standard location slot."""
        unmarked = [s for s in LOCATION_SLOTS if s not in self.locations]
        if not unmarked:
            print("[MarkGoals] All pre-defined locations already marked!")
            return

        print(f"\n[MarkGoals] Will mark: {', '.join(unmarked)}")
        print("For each location:")
        print("  1. In RViz, click 'Publish Point' tool")
        print("  2. Click on the location on the map")
        print("  3. Come back here and press Enter when done\n")

        for name in unmarked:
            if rospy.is_shutdown():
                break

            print(f"─── Marking: {name.upper()} ───")
            if name == "kitchen":
                print("  Click on the KITCHEN area (robot starting point)")
            else:
                table_num = name.replace("table_", "")
                print(f"  Click on TABLE {table_num} location on the map")

            with self.lock:
                self.pending_name = name

            # Wait for RViz click (up to 60 seconds)
            print("  Waiting for RViz click... (or press Enter to skip)")
            click_received = self._wait_for_click_or_enter(name, timeout=60)

            if not click_received:
                print(f"  Skipped {name}.")
                with self.lock:
                    self.pending_name = None
            else:
                # Optionally set yaw
                self._set_yaw_for_location(name)

    # ── Mark custom location ─────────────────────────────────────────────────
    def _mark_custom_location(self):
        """Mark a location with a custom name."""
        try:
            name = input("Enter location name (e.g. 'bar', 'exit'): ").strip()
        except (EOFError, KeyboardInterrupt):
            return

        if not name:
            print("Empty name, skipping.")
            return

        print(f"[MarkGoals] Click on '{name}' location in RViz...")

        with self.lock:
            self.pending_name = name

        click_received = self._wait_for_click_or_enter(name, timeout=60)
        if not click_received:
            with self.lock:
                self.pending_name = None
        else:
            self._set_yaw_for_location(name)

    # ── Manual coordinate entry ───────────────────────────────────────────────
    def _manual_entry(self):
        """Enter coordinates directly without RViz."""
        print("\n[MarkGoals] Manual entry mode.")
        print("Available slot names:", ", ".join(LOCATION_SLOTS))
        print("Or enter a custom name.\n")

        try:
            name = input("Location name: ").strip()
            if not name:
                return

            x_str = input(f"  x coordinate for '{name}': ").strip()
            y_str = input(f"  y coordinate for '{name}': ").strip()
            yaw_str = input(f"  yaw in degrees (0 = face right, 90 = face up) "
                            f"[default=0]: ").strip()

            x   = float(x_str)
            y   = float(y_str)
            yaw = float(yaw_str) if yaw_str else 0.0

            with self.lock:
                self.locations[name] = {'x': x, 'y': y, 'yaw': yaw}

            print(f"[MarkGoals] ✓ '{name}' set to ({x:.3f}, {y:.3f}, {yaw:.1f}°)")
            self._republish_markers(None)

        except ValueError as e:
            print(f"[MarkGoals] Invalid number: {e}")
        except (EOFError, KeyboardInterrupt):
            pass

    # ── Yaw setter ────────────────────────────────────────────────────────────
    def _set_yaw_for_location(self, name: str):
        """Optionally set the facing direction for an annotated location."""
        try:
            print(f"\n  Set yaw (facing direction) for '{name}':")
            print("   0   = Robot faces +x (East)")
            print("   90  = Robot faces +y (North)")
            print("  -90  = Robot faces -y (South)")
            print("  180  = Robot faces -x (West)")
            yaw_str = input("  Yaw degrees [default=0]: ").strip()
            yaw = float(yaw_str) if yaw_str else 0.0

            with self.lock:
                if name in self.locations:
                    self.locations[name]['yaw'] = yaw
            print(f"  Yaw set to {yaw:.1f}°")
        except (ValueError, EOFError, KeyboardInterrupt):
            pass

    # ── Wait for click or manual enter ────────────────────────────────────────
    def _wait_for_click_or_enter(self, name: str, timeout: int = 60) -> bool:
        """
        Block until either:
          - RViz click received for 'name', or
          - User presses Enter (skip)
        Returns True if click received, False if skipped/timed out.
        """
        received = threading.Event()

        # Watch for the location to appear in self.locations
        def watcher():
            start = time.time()
            while time.time() - start < timeout:
                with self.lock:
                    if name in self.locations and self.pending_name is None:
                        received.set()
                        return
                time.sleep(0.1)

        watcher_thread = threading.Thread(target=watcher, daemon=True)
        watcher_thread.start()

        # Also allow user to press Enter to skip
        input_thread_done = threading.Event()
        user_pressed_enter = [False]

        def input_watcher():
            try:
                input()   # Blocks until Enter
                user_pressed_enter[0] = True
                received.set()
            except Exception:
                pass
            input_thread_done.set()

        input_thread = threading.Thread(target=input_watcher, daemon=True)
        input_thread.start()

        received.wait(timeout=timeout + 1)

        with self.lock:
            click_done = (name in self.locations and
                          self.pending_name is None)

        return click_done and not user_pressed_enter[0]

    # ── Save to config ────────────────────────────────────────────────────────
    def _save_to_config(self):
        """Write marked locations back to fsm_map_config.yaml."""
        if not self.locations:
            print("[MarkGoals] No locations to save.")
            return

        if not os.path.exists(CONFIG_FILE):
            rospy.logwarn("[MarkGoals] Config file missing: %s", CONFIG_FILE)
            # Create minimal config
            config = {'locations': {}}
        else:
            with open(CONFIG_FILE, 'r') as f:
                config = yaml.safe_load(f) or {}

        # Update locations section
        config.setdefault('locations', {})
        with self.lock:
            for name, loc in self.locations.items():
                config['locations'][name] = [
                    round(loc['x'],   3),
                    round(loc['y'],   3),
                    round(loc['yaw'], 1)
                ]

        # Write back
        with open(CONFIG_FILE, 'w') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=True)

        rospy.loginfo("[MarkGoals] Saved %d locations to %s",
                      len(self.locations), CONFIG_FILE)

    # ── RViz marker publishing ────────────────────────────────────────────────
    def _republish_markers(self, event):
        """Publish all marked locations as colored cylinder markers in RViz."""
        marker_array = MarkerArray()

        with self.lock:
            locations_snapshot = dict(self.locations)

        for i, (name, loc) in enumerate(locations_snapshot.items()):
            # ── Cylinder marker at goal position ─────────────────────────
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp    = rospy.Time.now()
            m.ns              = "delivery_goals"
            m.id              = i
            m.type            = Marker.CYLINDER
            m.action          = Marker.ADD
            m.pose.position.x = loc['x']
            m.pose.position.y = loc['y']
            m.pose.position.z = 0.3     # Hover 30cm above ground
            m.pose.orientation.w = 1.0
            m.scale.x = 0.3   # 30cm diameter
            m.scale.y = 0.3
            m.scale.z = 0.6   # 60cm tall

            r, g, b = MARKER_COLORS.get(name, DEFAULT_COLOR)
            m.color = ColorRGBA(r=r, g=g, b=b, a=0.8)
            m.lifetime = rospy.Duration(0)    # Persist forever
            marker_array.markers.append(m)

            # ── Text label above marker ────────────────────────────────────
            t = Marker()
            t.header.frame_id = "map"
            t.header.stamp    = rospy.Time.now()
            t.ns              = "delivery_labels"
            t.id              = i + 1000
            t.type            = Marker.TEXT_VIEW_FACING
            t.action          = Marker.ADD
            t.pose.position.x = loc['x']
            t.pose.position.y = loc['y']
            t.pose.position.z = 0.8
            t.pose.orientation.w = 1.0
            t.scale.z = 0.25   # Text height 25cm
            t.color   = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            t.text    = name.upper()
            t.lifetime = rospy.Duration(0)
            marker_array.markers.append(t)

        self.marker_pub.publish(marker_array)

    # ── Summary print ─────────────────────────────────────────────────────────
    def _print_summary(self):
        print("\n" + "=" * 50)
        print("  Marked Delivery Locations:")
        print("=" * 50)
        with self.lock:
            for name, loc in self.locations.items():
                print(f"  {name:12s}: x={loc['x']:6.2f}  "
                      f"y={loc['y']:6.2f}  yaw={loc['yaw']:5.1f}°")
        print("=" * 50)
        print(f"\nConfig file: {CONFIG_FILE}")
        print("These locations are now used by fsm_map_fusion.py")
        print("You can re-run this script anytime to update them.\n")


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    try:
        marker = GoalPointMarker()
        # Small delay to let ROS connections establish
        time.sleep(1.0)
        marker.run_interactive_session()
    except rospy.ROSInterruptException:
        rospy.loginfo("[MarkGoals] Shutdown.")
    except KeyboardInterrupt:
        print("\n[MarkGoals] Interrupted.")


if __name__ == "__main__":
    main()

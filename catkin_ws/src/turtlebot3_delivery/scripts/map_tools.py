#!/usr/bin/env python3
"""
map_tools.py — Map Save / Load / Status Tool
=============================================
Usage:
  Save current map:   python3 map_tools.py save
  Load a saved map:   python3 map_tools.py load
  Check map status:   python3 map_tools.py status
  Auto-save loop:     python3 map_tools.py autosave

This script is called automatically by start_all.sh.
It saves maps to: ~/maps/restaurant_map.yaml

Dependencies:
  sudo apt install ros-noetic-map-server
"""

import os
import sys
import time
import subprocess
import shutil
from datetime import datetime

import rospy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


# ── Configuration ────────────────────────────────────────────────────────────
MAP_DIR      = os.path.expanduser("~/maps")
MAP_NAME     = "restaurant_map"
MAP_YAML     = os.path.join(MAP_DIR, MAP_NAME + ".yaml")
MAP_PGM      = os.path.join(MAP_DIR, MAP_NAME + ".pgm")
BACKUP_DIR   = os.path.join(MAP_DIR, "backups")


# ── Map Save ─────────────────────────────────────────────────────────────────
def save_map(name: str = MAP_NAME):
    """
    Save the current /map topic to disk using map_saver.
    Requires gmapping to be publishing a /map topic.
    Creates backup of previous map automatically.
    """
    os.makedirs(MAP_DIR,    exist_ok=True)
    os.makedirs(BACKUP_DIR, exist_ok=True)

    # Backup existing map if present
    if os.path.exists(MAP_YAML):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_base = os.path.join(BACKUP_DIR, f"{name}_{timestamp}")
        shutil.copy2(MAP_YAML, backup_base + ".yaml")
        if os.path.exists(MAP_PGM):
            shutil.copy2(MAP_PGM, backup_base + ".pgm")
        print(f"[MapTools] Existing map backed up to: {backup_base}.*")

    # Run map_saver
    output_path = os.path.join(MAP_DIR, name)
    print(f"[MapTools] Saving map to: {output_path}.*  ...", end=" ", flush=True)

    try:
        result = subprocess.run(
            ["rosrun", "map_server", "map_saver", "-f", output_path],
            timeout=15,
            capture_output=True,
            text=True
        )
        if result.returncode == 0:
            print("OK")
            print(f"[MapTools] Map saved:")
            print(f"  YAML: {output_path}.yaml")
            print(f"  PGM:  {output_path}.pgm")
            return True
        else:
            print(f"FAILED\n[MapTools] Error: {result.stderr}")
            return False
    except subprocess.TimeoutExpired:
        print("TIMEOUT\n[MapTools] map_saver took too long. "
              "Is gmapping running and /map topic published?")
        return False
    except FileNotFoundError:
        print("ERROR\n[MapTools] rosrun not found. Source ROS first:\n"
              "  source /opt/ros/noetic/setup.bash")
        return False


# ── Map Load ──────────────────────────────────────────────────────────────────
def load_map(yaml_path: str = MAP_YAML):
    """
    Launch map_server to serve the saved map as /map topic.
    Call this during navigation (not during mapping).
    """
    if not os.path.exists(yaml_path):
        print(f"[MapTools] ERROR: Map file not found: {yaml_path}")
        print("[MapTools] Run 'python3 map_tools.py save' after mapping first.")
        return False

    print(f"[MapTools] Loading map: {yaml_path}")
    print("[MapTools] Launching map_server... (leave this terminal open)")

    try:
        # Launch map_server as a subprocess (blocks, serves map continuously)
        subprocess.run(
            ["rosrun", "map_server", "map_server", yaml_path],
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"[MapTools] map_server failed: {e}")
        return False
    except KeyboardInterrupt:
        print("\n[MapTools] map_server stopped.")

    return True


# ── Map Status Check ─────────────────────────────────────────────────────────
def check_status():
    """
    Check if a map exists on disk and report its properties.
    Also checks if /map topic is being published.
    """
    print("=" * 50)
    print("   Map Status Report")
    print("=" * 50)

    # Check disk
    if os.path.exists(MAP_YAML):
        size_kb = os.path.getsize(MAP_YAML) / 1024
        mtime   = datetime.fromtimestamp(os.path.getmtime(MAP_YAML))
        print(f"[✓] Map file:     {MAP_YAML}")
        print(f"    YAML size:    {size_kb:.1f} KB")
        print(f"    Last saved:   {mtime.strftime('%Y-%m-%d %H:%M:%S')}")
    else:
        print(f"[✗] No map found at: {MAP_YAML}")
        print("    → Run mapping.launch then 'python3 map_tools.py save'")

    if os.path.exists(MAP_PGM):
        pgm_kb = os.path.getsize(MAP_PGM) / 1024
        print(f"[✓] PGM image:    {MAP_PGM}  ({pgm_kb:.0f} KB)")
    else:
        print(f"[✗] No PGM image at: {MAP_PGM}")

    # List backups
    if os.path.exists(BACKUP_DIR):
        backups = [f for f in os.listdir(BACKUP_DIR) if f.endswith(".yaml")]
        if backups:
            print(f"[✓] Backups:      {len(backups)} previous maps in {BACKUP_DIR}/")

    # Check /map topic
    print("\n[MapTools] Checking /map ROS topic (2 second timeout)...")
    try:
        rospy.init_node("map_status_checker", anonymous=True)
        msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=2.0)
        w, h = msg.info.width, msg.info.height
        res  = msg.info.resolution
        print(f"[✓] /map topic:   ACTIVE  ({w}x{h} cells, {res}m/cell)")
        print(f"    Real size:    {w*res:.1f}m x {h*res:.1f}m")
    except rospy.ROSException:
        print("[✗] /map topic:   NOT PUBLISHING")
        print("    → Start gmapping or map_server first")
    except Exception:
        print("[~] /map check:   Skipped (ROS not running)")

    print("=" * 50)


# ── Auto-save loop ───────────────────────────────────────────────────────────
def autosave_loop(interval_seconds: int = 60):
    """
    Continuously save the map every N seconds during mapping.
    Run in a separate terminal while driving the robot.
    Press Ctrl+C to stop.
    """
    print(f"[MapTools] Auto-save mode: saving every {interval_seconds}s")
    print("[MapTools] Press Ctrl+C to stop.\n")

    count = 0
    try:
        while True:
            count += 1
            print(f"\n[MapTools] Auto-save #{count} at "
                  f"{datetime.now().strftime('%H:%M:%S')}")
            save_map()
            time.sleep(interval_seconds)
    except KeyboardInterrupt:
        print(f"\n[MapTools] Auto-save stopped after {count} saves.")
        print(f"[MapTools] Final map: {MAP_YAML}")


# ── Entry point ───────────────────────────────────────────────────────────────
def main():
    if len(sys.argv) < 2:
        print("Usage: python3 map_tools.py [save | load | status | autosave]")
        print("")
        print("  save      — Save current /map topic to ~/maps/restaurant_map.*")
        print("  load      — Serve saved map as /map topic (for navigation)")
        print("  status    — Check if map exists and /map topic is active")
        print("  autosave  — Save map every 60 seconds during mapping session")
        sys.exit(1)

    cmd = sys.argv[1].lower()

    if cmd == "save":
        name = sys.argv[2] if len(sys.argv) > 2 else MAP_NAME
        success = save_map(name)
        sys.exit(0 if success else 1)

    elif cmd == "load":
        path = sys.argv[2] if len(sys.argv) > 2 else MAP_YAML
        load_map(path)

    elif cmd == "status":
        check_status()

    elif cmd == "autosave":
        interval = int(sys.argv[2]) if len(sys.argv) > 2 else 60
        autosave_loop(interval)

    else:
        print(f"[MapTools] Unknown command: '{cmd}'")
        print("Use: save | load | status | autosave")
        sys.exit(1)


if __name__ == "__main__":
    main()

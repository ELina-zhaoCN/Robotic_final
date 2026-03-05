#!/bin/bash
# =============================================================================
#  start_all.sh — One-Click Food Delivery System Launcher
#  TurtleBot3 Waffle — Restaurant FSM Delivery
# =============================================================================
#
#  USAGE:
#    First time (build map):     bash start_all.sh --map
#    Normal delivery operation:  bash start_all.sh
#    Annotate goal points:       bash start_all.sh --annotate
#    Check system status:        bash start_all.sh --status
#
#  PREREQUISITES:
#    1. ROS Noetic installed and sourced
#    2. TurtleBot3 bringup running on the robot
#    3. Map built (run with --map first)
#    4. catkin_make completed
#
#  NETWORK: Set ROBOT_IP below before running!
# =============================================================================

set -e   # Exit immediately on error

# ── USER CONFIGURATION — EDIT THESE ──────────────────────────────────────────
ROBOT_IP="192.168.1.100"         # ← Change to your robot's IP address
PC_IP="192.168.1.200"            # ← Change to your PC's IP address
MAP_FILE="$HOME/maps/restaurant_map.yaml"
TURTLEBOT3_MODEL="waffle"
CATKIN_WS="$HOME/catkin_ws"
PKG="turtlebot3_delivery"

# Terminal emulator (auto-detected below)
TERM_CMD=""
# ─────────────────────────────────────────────────────────────────────────────

# ── Colors for output ─────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
BLUE='\033[0;34m'; CYAN='\033[0;36m'; NC='\033[0m'

log_info()  { echo -e "${GREEN}[INFO]${NC}  $1"; }
log_warn()  { echo -e "${YELLOW}[WARN]${NC}  $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }
log_step()  { echo -e "\n${CYAN}══════════════════════════════════════${NC}"; \
              echo -e "${CYAN}  STEP: $1${NC}"; \
              echo -e "${CYAN}══════════════════════════════════════${NC}"; }

# ── Detect terminal emulator ──────────────────────────────────────────────────
detect_terminal() {
    if command -v gnome-terminal &>/dev/null; then
        TERM_CMD="gnome-terminal --"
    elif command -v xterm &>/dev/null; then
        TERM_CMD="xterm -e"
    elif command -v konsole &>/dev/null; then
        TERM_CMD="konsole -e"
    elif command -v xfce4-terminal &>/dev/null; then
        TERM_CMD="xfce4-terminal --command"
    else
        log_error "No supported terminal emulator found!"
        log_error "Install one: sudo apt install gnome-terminal"
        exit 1
    fi
    log_info "Using terminal: $TERM_CMD"
}

# ── Environment setup ─────────────────────────────────────────────────────────
setup_env() {
    source /opt/ros/noetic/setup.bash
    source "$CATKIN_WS/devel/setup.bash" 2>/dev/null || {
        log_error "catkin_ws not built! Run: cd $CATKIN_WS && catkin_make"
        exit 1
    }
    export TURTLEBOT3_MODEL="$TURTLEBOT3_MODEL"
    export ROS_MASTER_URI="http://${PC_IP}:11311"
    export ROS_HOSTNAME="$PC_IP"
    log_info "ROS environment configured."
    log_info "  ROS_MASTER_URI=$ROS_MASTER_URI"
    log_info "  TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
}

# ── Open a command in a new terminal window ───────────────────────────────────
open_terminal() {
    local title="$1"
    local cmd="$2"
    log_info "Opening terminal: $title"
    if [[ "$TERM_CMD" == "gnome-terminal --" ]]; then
        gnome-terminal --title="$title" -- bash -c "
            source /opt/ros/noetic/setup.bash
            source $CATKIN_WS/devel/setup.bash
            export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL
            export ROS_MASTER_URI=http://${PC_IP}:11311
            export ROS_HOSTNAME=$PC_IP
            echo 'Terminal: $title'
            echo '─────────────────────────'
            $cmd
            echo 'Process ended. Press Enter to close.'
            read
        " &
    else
        $TERM_CMD bash -c "
            source /opt/ros/noetic/setup.bash
            source $CATKIN_WS/devel/setup.bash
            export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL
            export ROS_MASTER_URI=http://${PC_IP}:11311
            export ROS_HOSTNAME=$PC_IP
            $cmd; read
        " &
    fi
    sleep 0.5
}

# ── Wait for ROS topic to be active ──────────────────────────────────────────
wait_for_topic() {
    local topic="$1"
    local timeout="${2:-30}"
    log_info "Waiting for topic: $topic (timeout: ${timeout}s)..."
    local start_time
    start_time=$(date +%s)
    while ! rostopic list 2>/dev/null | grep -q "^${topic}$"; do
        local now
        now=$(date +%s)
        if (( now - start_time > timeout )); then
            log_warn "Topic $topic not found after ${timeout}s. Continuing anyway."
            return 1
        fi
        sleep 1
    done
    log_info "Topic $topic is active."
    return 0
}

# ── Check if map exists ───────────────────────────────────────────────────────
check_map() {
    if [ ! -f "$MAP_FILE" ]; then
        log_error "Map not found: $MAP_FILE"
        log_error "Run mapping first: bash start_all.sh --map"
        exit 1
    fi
    log_info "Map found: $MAP_FILE"
}

# =============================================================================
#  MODE: STATUS CHECK
# =============================================================================
cmd_status() {
    log_step "System Status Check"
    setup_env

    echo ""
    echo "── ROS Core ──────────────────────────────────────"
    if rostopic list &>/dev/null 2>&1; then
        log_info "roscore: RUNNING"
    else
        log_warn "roscore: NOT RUNNING"
    fi

    echo ""
    echo "── Map File ──────────────────────────────────────"
    python3 "$CATKIN_WS/src/$PKG/scripts/map_tools.py" status

    echo ""
    echo "── Active ROS Topics ─────────────────────────────"
    rostopic list 2>/dev/null | grep -E "scan|amcl|cmd_vel|robot_state|delivery" \
        | while read t; do log_info "  $t"; done

    echo ""
    echo "── Test Commands ─────────────────────────────────"
    echo "  rostopic echo /robot_state"
    echo "  rostopic pub /delivery_task std_msgs/String \"data: 'table_1'\""
    echo "  rostopic echo /finish_task"
}

# =============================================================================
#  MODE: MAPPING (Phase 1 — Build the restaurant map)
# =============================================================================
cmd_mapping() {
    log_step "Phase 1: SLAM Mapping Mode"

    setup_env
    detect_terminal

    log_warn "IMPORTANT: This mode builds the restaurant map."
    log_warn "Drive the robot to cover ALL areas of the restaurant."
    log_warn "Save the map when done by pressing 'M' in the keyboard window."
    echo ""
    read -p "Press Enter when the robot is powered on and bringup is running..."

    # Create maps directory
    mkdir -p "$HOME/maps"
    log_info "Map will be saved to: $MAP_FILE"

    # Terminal 1: roscore
    log_step "Starting roscore"
    open_terminal "roscore" "roscore"
    sleep 2

    # Terminal 2: gmapping SLAM
    log_step "Starting gmapping SLAM"
    open_terminal "gmapping-SLAM" \
        "roslaunch $PKG mapping.launch"
    sleep 5

    # Wait for /map topic
    wait_for_topic "/map" 30

    # Terminal 3: Keyboard control for mapping
    log_step "Starting keyboard mapping controller"
    open_terminal "Keyboard-Mapping" \
        "cd $CATKIN_WS/src/$PKG && python3 scripts/keyboard_mapping.py"
    sleep 2

    # Terminal 4: Auto-save every 60 seconds
    log_step "Starting map auto-save (every 60s)"
    open_terminal "Map-AutoSave" \
        "cd $CATKIN_WS/src/$PKG && python3 scripts/map_tools.py autosave 60"

    echo ""
    echo -e "${GREEN}══════════════════════════════════════════════${NC}"
    echo -e "${GREEN}  MAPPING IS RUNNING!${NC}"
    echo -e "${GREEN}══════════════════════════════════════════════${NC}"
    echo ""
    echo "  1. Use the KEYBOARD MAPPING window to drive the robot"
    echo "  2. Cover ALL areas: kitchen, aisles, all table locations"
    echo "  3. Drive slowly for best map quality"
    echo "  4. Watch RViz — make sure all walls appear on the map"
    echo "  5. When done: press 'M' in keyboard window to save"
    echo ""
    echo "  Map will be saved to: $MAP_FILE"
    echo ""
    echo "  After saving, run the annotator:"
    echo "  bash start_all.sh --annotate"
    echo ""
    echo -e "${YELLOW}  Press Ctrl+C here to stop all mapping processes${NC}"
    wait
}

# =============================================================================
#  MODE: ANNOTATE (Phase 2 — Mark delivery locations)
# =============================================================================
cmd_annotate() {
    log_step "Phase 2: Goal Point Annotation"

    setup_env
    detect_terminal
    check_map

    # Start navigation stack to show the map
    open_terminal "roscore" "roscore"
    sleep 2

    open_terminal "Navigation-Stack" \
        "roslaunch $PKG navigation.launch map_file:=$MAP_FILE open_rviz:=true"
    sleep 8

    wait_for_topic "/amcl_pose" 30

    echo ""
    echo -e "${GREEN}══════════════════════════════════════════════${NC}"
    echo -e "${GREEN}  ANNOTATION MODE${NC}"
    echo -e "${GREEN}══════════════════════════════════════════════${NC}"
    echo ""
    echo "  1. RViz is open — set initial pose with '2D Pose Estimate'"
    echo "  2. The annotator will start now in this terminal"
    echo "  3. Follow the on-screen prompts to mark each table location"
    echo ""

    # Run annotator in THIS terminal (interactive)
    source /opt/ros/noetic/setup.bash
    source "$CATKIN_WS/devel/setup.bash"
    export TURTLEBOT3_MODEL="$TURTLEBOT3_MODEL"
    export ROS_MASTER_URI="http://${PC_IP}:11311"
    python3 "$CATKIN_WS/src/$PKG/scripts/mark_goal_points.py"
}

# =============================================================================
#  MODE: DELIVERY (Phase 3 — Normal operation)
# =============================================================================
cmd_delivery() {
    log_step "Phase 3: Food Delivery Operation"

    setup_env
    detect_terminal
    check_map

    echo ""
    log_info "Starting complete food delivery system..."
    echo ""

    # ── Step 1: roscore ───────────────────────────────────────────────────
    log_step "1/5  Starting roscore"
    open_terminal "① roscore" "roscore"
    sleep 3

    # ── Step 2: Navigation stack ──────────────────────────────────────────
    log_step "2/5  Starting navigation (AMCL + move_base)"
    open_terminal "② Navigation" \
        "roslaunch $PKG navigation.launch map_file:=$MAP_FILE open_rviz:=true"
    sleep 8

    wait_for_topic "/amcl_pose"    30
    wait_for_topic "/move_base"    30

    # ── Step 3: Obstacle detector ─────────────────────────────────────────
    log_step "3/5  Starting obstacle detector (RPLIDAR A1M8, 0.5m)"
    open_terminal "③ ObstacleDetector" \
        "rosrun $PKG obstacle_detector.py \
            _obstacle_threshold:=0.50 \
            _scan_angle_min_deg:=-180.0 \
            _scan_angle_max_deg:=180.0"
    sleep 2

    wait_for_topic "/obstacle_detected" 15

    # ── Step 4: Task manager ──────────────────────────────────────────────
    log_step "4/5  Starting task manager"
    open_terminal "④ TaskManager" \
        "rosrun $PKG task_manager.py"
    sleep 2

    # ── Step 5: FSM Map Fusion (main controller) ──────────────────────────
    log_step "5/5  Starting FSM Map Fusion controller"
    open_terminal "⑤ FSM-MapFusion" \
        "rosrun $PKG fsm_map_fusion.py"
    sleep 3

    wait_for_topic "/robot_state" 20

    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║   FOOD DELIVERY SYSTEM IS RUNNING!           ║${NC}"
    echo -e "${GREEN}╠══════════════════════════════════════════════╣${NC}"
    echo -e "${GREEN}║                                              ║${NC}"
    echo -e "${GREEN}║  1. In RViz: click '2D Pose Estimate'        ║${NC}"
    echo -e "${GREEN}║     Click on kitchen location on map         ║${NC}"
    echo -e "${GREEN}║     Drag to set orientation                  ║${NC}"
    echo -e "${GREEN}║                                              ║${NC}"
    echo -e "${GREEN}║  2. Send a delivery task:                    ║${NC}"
    echo -e "${GREEN}║                                              ║${NC}"
    echo -e "${GREEN}║  ${YELLOW}rostopic pub /delivery_task \\${GREEN}             ║${NC}"
    echo -e "${GREEN}║  ${YELLOW}  std_msgs/String \"data: 'table_1'\"${GREEN}       ║${NC}"
    echo -e "${GREEN}║                                              ║${NC}"
    echo -e "${GREEN}║  3. Monitor states:                          ║${NC}"
    echo -e "${GREEN}║  ${YELLOW}rostopic echo /robot_state${GREEN}                ║${NC}"
    echo -e "${GREEN}║                                              ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════════╝${NC}"
    echo ""
    echo "Available tables: table_1  table_2  table_3  table_4  table_5"
    echo ""

    # Keep script running to allow Ctrl+C cleanup
    echo -e "${YELLOW}Press Ctrl+C to stop all delivery processes.${NC}"
    trap 'cleanup' INT TERM
    wait
}

# ── Cleanup all background processes ─────────────────────────────────────────
cleanup() {
    echo ""
    log_info "Shutting down all nodes..."
    pkill -f "roslaunch" 2>/dev/null || true
    pkill -f "fsm_map_fusion"    2>/dev/null || true
    pkill -f "obstacle_detector" 2>/dev/null || true
    pkill -f "task_manager"      2>/dev/null || true
    pkill -f "navigation_control" 2>/dev/null || true
    sleep 1
    pkill -f "roscore" 2>/dev/null || true
    log_info "All processes stopped."
    exit 0
}

# =============================================================================
#  ENTRY POINT
# =============================================================================
echo ""
echo -e "${CYAN}╔══════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║   TurtleBot3 Waffle — Restaurant Delivery    ║${NC}"
echo -e "${CYAN}║   FSM + Map Fusion System                    ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════════╝${NC}"
echo ""

MODE="${1:-}"

case "$MODE" in
    "--map"      | "-m")  cmd_mapping  ;;
    "--annotate" | "-a")  cmd_annotate ;;
    "--status"   | "-s")  cmd_status   ;;
    ""           | "--delivery") cmd_delivery ;;
    *)
        echo "Usage: bash start_all.sh [MODE]"
        echo ""
        echo "Modes:"
        echo "  (no args)    Normal food delivery operation"
        echo "  --map        Build restaurant map with keyboard control"
        echo "  --annotate   Mark kitchen / table locations on map"
        echo "  --status     Check system health"
        echo ""
        echo "Typical workflow:"
        echo "  1. bash start_all.sh --map       (build map once)"
        echo "  2. bash start_all.sh --annotate  (mark tables once)"
        echo "  3. bash start_all.sh             (daily operation)"
        exit 1
        ;;
esac

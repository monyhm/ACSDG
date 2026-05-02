#!/bin/bash
# demo_scenario.sh — ACSDG scripted demo sequence for client presentation.
# Usage: bash ~/acsdg_ws/scripts/demo_scenario.sh
#
# Assumes full system is already running:
#   ros2 launch acsdg_bringup acsdg_full.launch.py

source /opt/ros/humble/setup.bash
source ~/acsdg_ws/install/setup.bash 2>/dev/null

BOLD='\033[1m'; CYAN='\033[0;36m'; GREEN='\033[0;32m'
YELLOW='\033[0;33m'; RED='\033[0;31m'; RESET='\033[0m'

banner() {
  echo ""
  echo -e "${CYAN}${BOLD}┌────────────────────────────────────────────────────┐${RESET}"
  echo -e "${CYAN}${BOLD}│  $1${RESET}"
  echo -e "${CYAN}${BOLD}└────────────────────────────────────────────────────┘${RESET}"
}

step() {
  echo -e "${GREEN}[$(date +%H:%M:%S)]${RESET} ${BOLD}$1${RESET}"
}

wait_msg() {
  local secs=$1; local msg=$2
  echo -e "${YELLOW}  ⏳ Waiting ${secs}s — ${msg}${RESET}"
  sleep "$secs"
}

pub() {
  local topic=$1; local type=$2; local data=$3
  ros2 topic pub --once "$topic" "$type" "$data" > /dev/null 2>&1
}

# ── Pre-flight check ──────────────────────────────────────────────────────────
banner "ACSDG DEMO SEQUENCE — CLIENT PRESENTATION"
echo ""
step "Verifying system is online..."

MISSING=""
for n in c2_engine_node enemy_driver_node mission_manager_node; do
  if ! ros2 node list 2>/dev/null | grep -q "/$n"; then
    MISSING="$MISSING /$n"
  fi
done
if [ -n "$MISSING" ]; then
  echo -e "${RED}[ERROR] Required nodes not running:${MISSING}${RESET}"
  echo "  Start the full system first:"
  echo "  ros2 launch acsdg_bringup acsdg_full.launch.py"
  exit 1
fi
echo -e "  ${GREEN}System online ✓${RESET}"

# ── Phase 1: Stabilise ───────────────────────────────────────────────────────
banner "PHASE 1 — SYSTEM STABILISATION"
step "Allowing system to reach steady state..."
wait_msg 3 "C2 and AI nodes initialising"

# ── Phase 2: Wave 1 ──────────────────────────────────────────────────────────
banner "PHASE 2 — WAVE 1: SPREAD FORMATION (3 drones, 5 m/s)"
step "Triggering Wave 1..."
pub /mission/wave_trigger std_msgs/msg/Bool "data: true"
echo -e "  ${GREEN}Wave 1 launched ✓${RESET}"
echo "  Expect: 3 slow drones in SPREAD formation"
echo "  Watch:  IDLE interceptors assigned, SATURATION/UNKNOWN swarm tactic"
wait_msg 15 "Interceptors engaging Wave 1 targets"

# ── Phase 3: Wave 2 ──────────────────────────────────────────────────────────
banner "PHASE 3 — WAVE 2: PINCER FORMATION (5 drones, 8 m/s)"
step "Triggering Wave 2..."
pub /mission/wave_trigger std_msgs/msg/Bool "data: true"
echo -e "  ${GREEN}Wave 2 launched ✓${RESET}"
echo "  Expect: 5 drones in PINCER formation, fleet splits to cover vectors"
echo "  Watch:  PINCER tactic detected → SPLIT FLEET response"
wait_msg 20 "Engaging Wave 2 — observing AI tactic classification"

# ── Phase 4: Difficulty increase ─────────────────────────────────────────────
banner "PHASE 4 — ESCALATION: DIFFICULTY → 0.8"
step "Increasing mission difficulty to 0.8..."
pub /mission/difficulty std_msgs/msg/Float32 "data: 0.8"
echo -e "  ${GREEN}Difficulty raised ✓${RESET}"
echo "  Wave speeds and drone counts will increase"
wait_msg 3 "Difficulty applied"

# ── Phase 5: Wave 3 ──────────────────────────────────────────────────────────
banner "PHASE 5 — WAVE 3: SATURATION ATTACK (8 drones, 12+ m/s)"
step "Triggering Wave 3 — SATURATION..."
pub /mission/wave_trigger std_msgs/msg/Bool "data: true"
echo -e "  ${GREEN}Wave 3 launched ✓${RESET}"
echo "  Expect: 8 fast drones in tight cluster"
echo "  Watch:  SATURATION tactic → ACTIVATE ALL INTERCEPTORS"
echo "  Watch:  AI confidence bar fills, all 4 interceptors dispatch"
wait_msg 20 "Peak engagement — AI classifying swarm, interceptors pursuing"

# ── Summary ───────────────────────────────────────────────────────────────────
banner "DEMO SEQUENCE COMPLETE"
echo ""
step "Fetching mission summary..."

# Get final mission status
MISSION=$(ros2 topic echo /mission/status acsdg_msgs/msg/MissionStatus \
          --once --no-daemon 2>/dev/null | head -20)

if [ -n "$MISSION" ]; then
  echo "$MISSION" | grep -E "wave|detected|neutral|breach|cost|response" | \
    sed 's/^/  /'
else
  echo "  (Mission status unavailable — check /mission/status manually)"
fi

echo ""
echo -e "${GREEN}${BOLD}╔══════════════════════════════════════════════╗${RESET}"
echo -e "${GREEN}${BOLD}║  DEMO SEQUENCE COMPLETE                      ║${RESET}"
echo -e "${GREEN}${BOLD}║                                              ║${RESET}"
echo -e "${GREEN}${BOLD}║  Dashboard: http://172.19.140.70:3000        ║${RESET}"
echo -e "${GREEN}${BOLD}╚══════════════════════════════════════════════╝${RESET}"
echo ""

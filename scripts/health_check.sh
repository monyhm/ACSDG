#!/bin/bash
# health_check.sh — ACSDG full system health check
# Usage: bash ~/acsdg_ws/scripts/health_check.sh

source /opt/ros/humble/setup.bash
source ~/acsdg_ws/install/setup.bash 2>/dev/null

PASS=0; FAIL=0
T_TIMEOUT=8   # seconds to wait for topic Hz check

GREEN='\033[0;32m'; RED='\033[0;31m'; CYAN='\033[0;36m'
BOLD='\033[1m'; RESET='\033[0m'

pass() { echo -e "  ${GREEN}[PASS]${RESET} $1"; ((PASS++)); }
fail() { echo -e "  ${RED}[FAIL]${RESET} $1"; ((FAIL++)); }

section() { echo -e "\n${CYAN}${BOLD}━━ $1 ━━${RESET}"; }

echo -e "${BOLD}"
echo "╔══════════════════════════════════════════════════╗"
echo "║       ACSDG SYSTEM HEALTH CHECK                 ║"
echo "╚══════════════════════════════════════════════════╝"
echo -e "${RESET}"

# ── 1. ROS2 Nodes ─────────────────────────────────────────────────────────────
section "ROS2 NODES"

REQUIRED_NODES=(
  "radar_node"
  "rf_node"
  "sensor_fusion_node"
  "c2_engine_node"
  "interceptor_manager_node"
  "mission_manager_node"
  "interceptor_controller_1"
  "interceptor_controller_2"
  "interceptor_controller_3"
  "interceptor_controller_4"
  "threat_predictor_node"
  "swarm_classifier_node"
  "learning_node"
  "enemy_driver_node"
  "rosbridge_websocket"
  "dashboard_server_node"
)

for node in "${REQUIRED_NODES[@]}"; do
  if ros2 node list 2>/dev/null | grep -q "/$node"; then
    pass "/$node"
  else
    fail "/$node — NOT RUNNING"
  fi
done

# ── 2. Key Topics + Hz ────────────────────────────────────────────────────────
section "TOPIC HEALTH (Hz)"

check_topic() {
  local topic=$1
  local min_hz=$2
  local label=${3:-$topic}

  # Sample for T_TIMEOUT seconds
  local hz
  hz=$(timeout $T_TIMEOUT ros2 topic hz "$topic" 2>/dev/null \
       | grep "average rate" | tail -1 | awk '{print $3}' | tr -d ':')

  if [ -z "$hz" ]; then
    fail "$label — NO DATA"
  else
    # Float comparison via awk (avoids integer truncation of e.g. 0.999 Hz)
    if awk "BEGIN { exit !(${hz}+0 >= ${min_hz}+0) }"; then
      pass "$label — ${hz} Hz"
    else
      fail "$label — ${hz} Hz (expected ≥${min_hz})"
    fi
  fi
}

# check_topic_pub: pass if topic has ≥1 publisher (for event-driven topics).
check_topic_pub() {
  local topic=$1
  local label=${2:-$topic}
  local pubs
  pubs=$(ros2 topic info "$topic" 2>/dev/null | grep "Publisher count" | awk '{print $3}')
  if [ "${pubs:-0}" -ge 1 ] 2>/dev/null; then
    pass "$label — publisher present"
  else
    fail "$label — NO PUBLISHER"
  fi
}

check_topic "/sensors/fusion/targets"    0.9  "/sensors/fusion/targets"
check_topic "/c2/threat_scores"          0.9  "/c2/threat_scores"
check_topic_pub "/c2/engagement_orders"          "/c2/engagement_orders (event — publisher check)"
check_topic "/ai/threat_predictions"     0.9  "/ai/threat_predictions"
check_topic "/ai/swarm_classification"   0.9  "/ai/swarm_classification"
check_topic "/interceptors/fleet_status" 0.9  "/interceptors/fleet_status"
check_topic "/mission/status"            0.9  "/mission/status"
check_topic "/learning/status"           0.9  "/learning/status"

# ── 3. Gazebo World ───────────────────────────────────────────────────────────
section "GAZEBO WORLD"

if gz topic -l 2>/dev/null | grep -q "military_base"; then
  pass "/world/military_base — LOADED"
  DRONE_COUNT=$(gz topic -l 2>/dev/null | grep "/model/enemy_" | grep "/odometry$" | wc -l)
  if [ "$DRONE_COUNT" -ge 4 ]; then
    pass "Enemy drone topics — ${DRONE_COUNT}/4 found"
  else
    fail "Enemy drone topics — only ${DRONE_COUNT}/4 found"
  fi
  INT_COUNT=$(gz topic -l 2>/dev/null | grep "/model/interceptor_" | grep "/odometry$" | wc -l)
  if [ "$INT_COUNT" -ge 4 ]; then
    pass "Interceptor topics — ${INT_COUNT}/4 found"
  else
    fail "Interceptor topics — only ${INT_COUNT}/4 found"
  fi
else
  fail "Gazebo world 'military_base' — NOT RUNNING"
  fail "Enemy drone topics — N/A"
  fail "Interceptor topics — N/A"
fi

# ── 4. Dashboard / rosbridge ──────────────────────────────────────────────────
section "NETWORK SERVICES"

# Dashboard HTTP
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" --connect-timeout 2 http://localhost:3000/ 2>/dev/null)
if [ "$HTTP_CODE" = "200" ]; then
  pass "Dashboard HTTP http://localhost:3000 — HTTP $HTTP_CODE"
else
  fail "Dashboard HTTP http://localhost:3000 — HTTP ${HTTP_CODE:-UNREACHABLE}"
fi

# rosbridge WebSocket (check if port is open)
if timeout 2 bash -c 'echo > /dev/tcp/localhost/9090' 2>/dev/null; then
  pass "rosbridge WebSocket ws://localhost:9090 — PORT OPEN"
else
  fail "rosbridge WebSocket ws://localhost:9090 — NOT REACHABLE"
fi

# ── 5. Summary ────────────────────────────────────────────────────────────────
TOTAL=$((PASS + FAIL))
echo ""
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
echo -e "  RESULT: ${GREEN}${PASS} PASS${RESET}  /  ${RED}${FAIL} FAIL${RESET}  /  ${TOTAL} total"
if [ "$FAIL" -eq 0 ]; then
  echo -e "  ${GREEN}${BOLD}ALL SYSTEMS NOMINAL — READY FOR DEMO${RESET}"
else
  echo -e "  ${RED}${BOLD}SYSTEM DEGRADED — $FAIL check(s) failed${RESET}"
fi
echo -e "${BOLD}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${RESET}"
echo ""

exit $FAIL

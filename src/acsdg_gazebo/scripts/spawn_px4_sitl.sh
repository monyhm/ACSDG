#!/bin/bash
# spawn_px4_sitl.sh — Launch N PX4 SITL instances for ACSDG simulation.
#
# Usage:
#   spawn_px4_sitl.sh <enemy_count> <interceptor_count>
#
# Prerequisites:
#   PX4-Autopilot must be compiled:
#     cd ~/PX4-Autopilot && make px4_sitl_default
#
# Each instance gets:
#   MAV_SYS_ID:  i+1
#   UDP GCS port: 14550 + i  (QGC connects to 14550)
#   UDP API port: 14540 + i  (MAVROS connects here)
#   Simulation port: 4560 + i
#
# Enemy drones (IDs 1..N_ENEMY):
#   Spawn at r=300m, evenly spaced around perimeter, alt=30m
#
# Interceptors (IDs N_ENEMY+1..N_ENEMY+N_INTER):
#   Spawn at defense posts (corners, r=177m), alt=5m

set -e

PX4_ROOT="${PX4_ROOT:-$HOME/PX4-Autopilot}"
PX4_BIN="$PX4_ROOT/build/px4_sitl_default/bin/px4"
PX4_RCDIR="$PX4_ROOT/ROMFS/px4fmu-common"

N_ENEMY="${1:-4}"
N_INTER="${2:-4}"

# ── Validation ────────────────────────────────────────────────────────────────
if [ ! -f "$PX4_BIN" ]; then
  echo "[ERROR] PX4 binary not found at $PX4_BIN"
  echo "        Build PX4 first:"
  echo "        cd $PX4_ROOT && make px4_sitl_default"
  exit 1
fi

source "$PX4_ROOT/Tools/simulation/gz/setup_gz.bash" 2>/dev/null || true

echo "[ACSDG] Spawning $N_ENEMY enemy + $N_INTER interceptor PX4 SITL instances"

# ── Spawn helper ──────────────────────────────────────────────────────────────
spawn_instance() {
  local INSTANCE=$1
  local PX4_X=$2
  local PX4_Y=$3
  local PX4_Z=$4
  local MODEL=$5

  local SYS_ID=$((INSTANCE + 1))
  local UDP_GCS=$((14550 + INSTANCE))
  local UDP_API=$((14540 + INSTANCE))
  local SIM_PORT=$((4560 + INSTANCE))
  local INST_DIR="/tmp/px4_instance_${INSTANCE}"

  mkdir -p "$INST_DIR"

  echo "[ACSDG] Instance $INSTANCE: SYS_ID=$SYS_ID  pos=($PX4_X,$PX4_Y,$PX4_Z)  api_port=$UDP_API"

  PX4_SIM_MODEL="$MODEL" \
  PX4_INSTANCE=$INSTANCE \
  PX4_SYS_AUTOSTART=4001 \
  "$PX4_BIN" \
    -i "$INSTANCE" \
    -d \
    -s "$PX4_RCDIR/init.d-posix/rcS" \
    -w "$INST_DIR" \
    -e "param set MAV_SYS_ID $SYS_ID" \
    -e "param set SIM_GZ_PX4_INSTANCE $INSTANCE" \
    -e "param save" \
    > "/tmp/px4_instance_${INSTANCE}.log" 2>&1 &

  echo $! > "/tmp/px4_instance_${INSTANCE}.pid"
  echo "[ACSDG] Instance $INSTANCE PID: $(cat /tmp/px4_instance_${INSTANCE}.pid)"
  sleep 0.5
}

# ── Enemy drones — perimeter at r=300m, evenly spaced ────────────────────────
PI=3.14159265358979
for i in $(seq 0 $((N_ENEMY - 1))); do
  ANGLE=$(echo "scale=6; $i * 2 * $PI / $N_ENEMY" | bc)
  X=$(echo "scale=2; 300 * c($ANGLE)" | bc -l)
  Y=$(echo "scale=2; 300 * s($ANGLE)" | bc -l)
  Z=30
  spawn_instance $i "$X" "$Y" "$Z" "x500"
done

# ── Interceptor drones — defense posts at corners ─────────────────────────────
INTER_POSITIONS=(
  "177 177 5"
  "-177 177 5"
  "177 -177 5"
  "-177 -177 5"
)
for i in $(seq 0 $((N_INTER - 1))); do
  INST=$((N_ENEMY + i))
  POS="${INTER_POSITIONS[$i]}"
  X=$(echo $POS | awk '{print $1}')
  Y=$(echo $POS | awk '{print $2}')
  Z=$(echo $POS | awk '{print $3}')
  spawn_instance $INST "$X" "$Y" "$Z" "x500"
done

echo ""
echo "[ACSDG] All instances spawned."
echo "        MAVROS connects to UDP ports: 14541-$((14540 + N_ENEMY + N_INTER))"
echo "        Kill all: cat /tmp/px4_instance_*.pid | xargs kill"
echo ""

# Keep script alive — Ctrl+C kills all children
trap 'echo "[ACSDG] Shutting down PX4 instances..."; kill $(cat /tmp/px4_instance_*.pid 2>/dev/null) 2>/dev/null; exit 0' INT TERM
wait

#!/usr/bin/env bash
set -euo pipefail

WS="${CATKIN_WS:-$HOME/catkin_ws}"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

rospack find highway_adas >/dev/null
rospack find highway_adas_ros >/dev/null
rospack find gadis_perception_msgs >/dev/null

MASTER_STARTED=0
if ! rostopic list >/dev/null 2>&1; then
  roscore >/tmp/highway_adas_roscore.log 2>&1 &
  MASTER_PID=$!
  MASTER_STARTED=1
  sleep 2
fi

LAUNCH_PID=""
cleanup() {
  if [[ -n "$LAUNCH_PID" ]]; then
    kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
    wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  fi
  if [[ "$MASTER_STARTED" -eq 1 ]]; then
    kill -INT "$MASTER_PID" >/dev/null 2>&1 || true
    wait "$MASTER_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

for scenario in empty rear_blocked front_follow; do
  LOG="/tmp/highway_adas_${scenario}.log"
  echo "=== $scenario ==="
  roslaunch highway_adas_ros synthetic_test.launch scenario:="$scenario" \
    >"$LOG" 2>&1 &
  LAUNCH_PID=$!
  sleep 0.5

  if ! rosrun highway_adas_ros verify_synthetic.py \
      --scenario "$scenario" --timeout 40; then
    echo "--- roslaunch log: $LOG ---"
    tail -n 120 "$LOG"
    exit 1
  fi

  kill -INT "$LAUNCH_PID" >/dev/null 2>&1 || true
  wait "$LAUNCH_PID" >/dev/null 2>&1 || true
  LAUNCH_PID=""
  sleep 2
done

echo "ALL SYNTHETIC HIGHWAY ADAS SCENARIOS PASSED"

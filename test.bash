#!/bin/bash -xv
# SPDX-FileCopyrightText: 2025 Yuken Ro
# SPDX-License-Identifier: BSD-3-Clause

set -e

set +u
: "${AMENT_TRACE_SETUP_FILES:=}"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
set -u

export LANG=C.UTF-8
export LC_ALL=C.UTF-8

ng() { echo "NG at line $1"; res=1; }
res=0

pkill -f "wear_advisor.*wear_" 2>/dev/null || true
sleep 0.2

ros2 run wear_advisor wear_node --ros-args -r __node:=wear_node_test &
NODE_PID=$!
sleep 0.8

run_case() {
  local temp="$1"
  local e1="$2"
  local e2="$3"
  local e3="$4"

  echo "==== REQ: ${temp} ===="

  tmp="$(mktemp)"
  ros2 topic echo -n 1 /wear_response > "$tmp" 2>/dev/null &
  ECHO_PID=$!
  sleep 0.2

  echo "$temp" | ros2 run wear_advisor wear_pub > /dev/null

  wait "$ECHO_PID" 2>/dev/null || true

  out="$(cat "$tmp")"
  rm -f "$tmp"

  echo "$out"

  echo "$out" | grep -Fq "$e1" || ng $LINENO
  echo "$out" | grep -Fq "$e2" || ng $LINENO
  echo "$out" | grep -Fq "$e3" || ng $LINENO
}

# 期待値（あなたのロジックに合わせて）
run_case "3"  "厚手" "長ズボン" "コート"
run_case "8"  "長袖" "長ズボン" "コート"
run_case "15" "長袖" "長ズボン" "薄手ジャケット"
run_case "22" "長袖" "長ズボン" "なし"
run_case "28" "半袖" "短パン" "なし"

kill "$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true

[ "$res" = 0 ] && echo "OK"
exit "$res"

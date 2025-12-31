#!/bin/bash -xv
# SPDX-FileCopyrightText: 2025 ken116610
# SPDX-License-Identifier: BSD-3-Clause

ng () {
  echo "NG at line $1"
  res=1
}

res=0

cd "$(dirname "$0")"
cd ~/ros2_ws
source install/setup.bash

ros2 run wear_advisor wear_node &
NODE_PID=$!
sleep 0.5

run_case () {
    req="$1"
    expected="$2"

    tmp=$(mktemp)

    ros2 topic echo -n 1 /wear_response > "$tmp" 2>/dev/null &
    ECHO_PID=$!

    sleep 0.2

    ros2 topic pub -1 /wear_request std_msgs/msg/String "{data: '$req'}" > /dev/null

    wait $ECHO_PID 2>/dev/null || true

    out=$(cat "$tmp")
    rm -f "$tmp"

    echo "$out" | grep -q "$expected" || ng $LINENO
}


run_case "28 sunny" "トップス: 半袖"
run_case "15 rain" "傘"
run_case "3 snow" "コート"

kill $NODE_PID
wait $NODE_PID 2>/dev/null || true

[ "$res" = 0 ] && echo OK
exit "$res"


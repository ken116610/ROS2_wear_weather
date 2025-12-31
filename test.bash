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

# start node
ros2 run wear_advisor wear_node &
NODE_PID=$!
sleep 0.5

# helper: publish then capture one response
run_case () {
  req="$1"
  expected="$2"

  # start echo (capture 1 message)
  out=$( (ros2 topic echo -n 1 /wear_response 2>/dev/null &) ; \
        sleep 0.1 ; \
        ros2 topic pub -1 /wear_request std_msgs/msg/String "{data: '$req'}" >/dev/null ; \
        sleep 0.5 ; \
        ros2 topic echo -n 1 /wear_response 2>/dev/null || true )

  echo "$out" | grep -q "$expected" || ng $LINENO
}

run_case "28 sunny" "トップス: 半袖"
run_case "15 rain" "傘"
run_case "3 snow" "コート"

kill $NODE_PID
wait $NODE_PID 2>/dev/null || true

[ "$res" = 0 ] && echo OK
exit "$res"


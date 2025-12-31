#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Yuken Ro
# SPDX-License-Identifier: BSD-3-Clause

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main() -> None:
    # 入力：引数があればそれ、なければstdin
    if len(sys.argv) >= 2:
        text = sys.argv[1]
    else:
        text = sys.stdin.read()

    text = (text or "").strip().split()[0] if (text or "").strip() else ""
    if not text:
        print("usage: echo '15' | ros2 run wear_advisor wear_pub", file=sys.stderr)
        return

    rclpy.init()
    node = Node("wear_pub")
    pub = node.create_publisher(String, "wear_request", 10)

    msg = String()
    msg.data = text

    # 取りこぼし防止で数回投げる
    for _ in range(3):
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


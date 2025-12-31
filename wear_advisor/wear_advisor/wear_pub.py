#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Yuken Ro
# SPDX-License-Identifier: BSD-3-Clause

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WearPub(Node):
    def __init__(self, payload: str):
        super().__init__("wear_pub")
        pub = self.create_publisher(String, "wear_request", 10)
        m = String()
        m.data = payload
        pub.publish(m)

def main() -> None:
    text = sys.stdin.read().strip()
    rclpy.init()
    node = WearPub(text)
    rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()


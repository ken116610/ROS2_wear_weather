#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Yuken Ro
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class WearPrint(Node):
    def __init__(self):
        super().__init__("wear_print")
        self.sub = self.create_subscription(String, "wear_response", self.cb, 10)

    def cb(self, msg: String) -> None:
        print(msg.data)
        rclpy.shutdown()

def main() -> None:
    rclpy.init()
    node = WearPrint()
    rclpy.spin(node)
    node.destroy_node()


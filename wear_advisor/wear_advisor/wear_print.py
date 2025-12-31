#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class WearPrint(Node):
    def __init__(self) -> None:
        super().__init__("wear_print")
        self.sub = self.create_subscription(String, "wear_response", self.cb, 10)

    def cb(self, msg: String) -> None:
        print(msg.data)
        rclpy.shutdown()


def main() -> None:
    rclpy.init()
    node = WearPrint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()


if __name__ == "__main__":
    main()


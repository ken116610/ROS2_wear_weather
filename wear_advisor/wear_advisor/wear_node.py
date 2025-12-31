#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Yuken Ro
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def parse_temp(text: str) -> int:
    s = text.strip()
    if not s:
        raise ValueError("empty")
    if not s.lstrip("-").isdigit():
        raise ValueError("not int")
    temp = int(s)
    if temp < -30 or temp > 50:
        raise ValueError("out of range")
    return temp


def suggest(temp: int) -> str:
    if temp <= 5:
        tops = "厚手（ニット等）"
        bottoms = "長ズボン"
        outer = "コート"
    elif temp <= 11:
        tops = "長袖"
        bottoms = "長ズボン"
        outer = "コート"
    elif temp <= 18:
        tops = "長袖"
        bottoms = "長ズボン"
        outer = "薄手ジャケット"
    elif temp <= 24:
        tops = "長袖（薄手）"
        bottoms = "長ズボン"
        outer = "なし"
    else:
        tops = "半袖"
        bottoms = "短パン（寒ければ長ズボンでもOK）"
        outer = "なし"

    return f"トップス : {tops}\nボトムス : {bottoms}\nアウター : {outer}"


class WearNode(Node):
    def __init__(self) -> None:
        super().__init__("wear_node")
        self.pub = self.create_publisher(String, "wear_response", 10)
        self.sub = self.create_subscription(String, "wear_request", self.cb, 10)

    def cb(self, msg: String) -> None:
        out = String()
        try:
            temp = parse_temp(msg.data)
            out.data = suggest(temp)
        except Exception:
            out.data = "Error: invalid input (example: 15)"
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = WearNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


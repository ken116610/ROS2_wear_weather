#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Yuken Ro
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys


def die(msg: str) -> None:
    print(msg, file=sys.stderr)
    raise ValueError(msg)


def parse_request(text: str) -> tuple[int, str]:
    # expect: "<temp> <weather>"
    parts = text.strip().split()
    if len(parts) != 2:
        die("Error: input must be '<temp> <weather>' e.g. '15 rain'")

    t_s, w = parts[0], parts[1].lower()
    if not (t_s.lstrip("-").isdigit()):
        die("Error: temp must be an integer")

    temp = int(t_s)
    if temp < -30 or temp > 50:
        die("Error: temp out of range (-30..50)")

    weathers = {"sunny", "cloudy", "rain", "snow"}
    if w not in weathers:
        die("Error: weather must be one of sunny/cloudy/rain/snow")

    return temp, w


def suggest(temp: int, weather: str) -> str:
    # base by temperature
    if temp <= 5:
        tops = "厚手（ニット等）"
        bottoms = "長ズボン"
        outer = "コート"
        items = []
    elif temp <= 11:
        tops = "長袖"
        bottoms = "長ズボン"
        outer = "コート"
        items = []
    elif temp <= 18:
        tops = "長袖"
        bottoms = "長ズボン"
        outer = "薄手ジャケット"
        items = []
    elif temp <= 24:
        tops = "長袖（薄手）"
        bottoms = "長ズボン"
        outer = "なし"
        items = []
    else:
        tops = "半袖"
        bottoms = "短パン（寒ければ長ズボンでもOK）"
        outer = "なし"
        items = ["飲み物"]

    # add by weather
    if weather == "rain":
        items.append("傘")
        if outer == "なし":
            outer = "薄手ジャケット"
    elif weather == "snow":
        items.append("手袋")
        if outer != "コート":
            outer = "コート"
    # sunny/cloudy: no change

    items_text = "、".join(items) if items else "なし"
    return f"トップス: {tops}\nボトムス: {bottoms}\nアウター: {outer}\n持ち物: {items_text}"


class WearNode(Node):
    def __init__(self) -> None:
        super().__init__("wear_node")
        self.pub = self.create_publisher(String, "wear_response", 10)
        self.sub = self.create_subscription(String, "wear_request", self.cb, 10)

    def cb(self, msg: String) -> None:
        try:
            temp, weather = parse_request(msg.data)
            out = String()
            out.data = suggest(temp, weather)
            self.pub.publish(out)
        except ValueError:
            # publish error to response too (optional)
            out = String()
            out.data = "Error: invalid input"
            self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = WearNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


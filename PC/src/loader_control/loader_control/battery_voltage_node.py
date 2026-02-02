#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import os
import time


class BatteryVoltageNode(Node):
    def __init__(self, name: str):
        super().__init__(name)
        self.subscription = self.create_subscription(
            Float32,
            "/battery_voltage",
            self.listener_callback,
            10,
        )
        self.latest_voltage = None
        self.low_voltage_threshold = 18.1
        self.low_voltage_duration_s = 120
        self.low_voltage_since = None
        self.shutdown_requested = False

    def listener_callback(self, msg: Float32) -> None:
        self.latest_voltage = msg.data
        now = time.monotonic()

        if msg.data < self.low_voltage_threshold:
            if self.low_voltage_since is None:
                self.low_voltage_since = now

            if (
                not self.shutdown_requested
                and (now - self.low_voltage_since) >= self.low_voltage_duration_s
            ):
                self.shutdown_requested = True
                os.system("shutdown now -h")
        else:
            self.low_voltage_since = None


def main(args=None):
    rclpy.init(args=args)
    node = BatteryVoltageNode("battery_voltage_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

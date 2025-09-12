#!/usr/bin/env python3
"""Minimal MAVROS GPS Bridge - Just works™"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsBridge(Node):
    def __init__(self):
        super().__init__('gps_bridge')
        self.pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.sub = self.create_subscription(
            NavSatFix, 
            '/mavros/global_position/raw/fix',
            lambda m: self.pub.publish(m), 
            10
        )
        self.get_logger().info('GPS Bridge active')


def main():
    rclpy.init()
    rclpy.spin(GpsBridge())


if __name__ == '__main__':
    main()
#!/usr/bin/env python3
"""
测量跨机器 ROS 2 话题延迟。

用法（在 Follower 车上运行）:
  python3 measure_cross_machine_delay.py --topic /virtual_leader/odometry/filtered --duration 60
  python3 measure_cross_machine_delay.py --topic /robot1/odometry/filtered --duration 120 --csv /tmp/delay.csv
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
import argparse
import time
import csv
import os


class DelayMeter(Node):
    def __init__(self, topic, duration, csv_path):
        super().__init__('delay_meter')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(Odometry, topic, self.cb, qos)
        self.delays = []
        self.start_time = time.time()
        self.duration = duration
        self.csv_path = csv_path

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        header_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        delay = now - header_t
        self.delays.append((now, delay))

        if delay > 0.1:
            self.get_logger().warn(f'High delay: {delay*1000:.1f} ms')

    def print_stats(self):
        if not self.delays:
            self.get_logger().error('No messages received! Check topic name and network.')
            return
        d = [x[1] for x in self.delays]
        d.sort()
        n = len(d)
        avg = sum(d) / n
        p50 = d[n // 2]
        p95 = d[int(n * 0.95)]
        p99 = d[int(n * 0.99)]

        self.get_logger().info(
            f'Count={n} | '
            f'avg={avg*1000:.1f}ms | '
            f'P50={p50*1000:.1f}ms | '
            f'P95={p95*1000:.1f}ms | '
            f'P99={p99*1000:.1f}ms | '
            f'max={max(d)*1000:.1f}ms | '
            f'min={min(d)*1000:.1f}ms'
        )

        if self.csv_path:
            with open(self.csv_path, 'w') as f:
                w = csv.writer(f)
                w.writerow(['timestamp', 'delay_ms'])
                for ts, delay in self.delays:
                    w.writerow([ts, delay * 1000])
            self.get_logger().info(f'Saved to {self.csv_path}')


def main():
    rclpy.init()
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/virtual_leader/odometry/filtered')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--csv', default='')
    args = parser.parse_args()

    node = DelayMeter(args.topic, args.duration, args.csv)

    try:
        rclpy.spin_once(node, timeout_sec=args.duration)
    except KeyboardInterrupt:
        pass

    node.print_stats()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

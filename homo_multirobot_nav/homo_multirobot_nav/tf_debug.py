#!/usr/bin/env python3
"""Diagnose TF lookup for laser_link -> base_footprint.

Run this alongside the sim to test whether a regular (non-lifecycle) node
can look up the transform that AMCL's MessageFilter needs.
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


class TfDebug(Node):
    def __init__(self):
        super().__init__("tf_debug")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(2.0, self._check)

    def _check(self):
        now = self.get_clock().now()
        self.get_logger().info(f"Clock now: {now.nanoseconds / 1e9:.3f}")
        try:
            t = self.tf_buffer.lookup_transform(
                "robot1_base_footprint",
                "robot1_laser_link",
                rclpy.time.Time(),
            )
            self.get_logger().info(
                f"OK: laser->base = ({t.transform.translation.x:.3f}, "
                f"{t.transform.translation.y:.3f}, {t.transform.translation.z:.3f})"
            )
        except Exception as e:
            self.get_logger().error(f"laser_link -> base_footprint FAILED: {e}")

        try:
            t = self.tf_buffer.lookup_transform(
                "robot1_odom",
                "robot1_base_footprint",
                rclpy.time.Time(),
            )
            self.get_logger().info(
                f"OK: base->odom = ({t.transform.translation.x:.3f}, "
                f"{t.transform.translation.y:.3f}, {t.transform.translation.z:.3f})"
            )
        except Exception as e:
            self.get_logger().error(f"base_footprint -> odom FAILED: {e}")


def main():
    rclpy.init()
    node = TfDebug()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""Pre-transform laser scans from laser_link to base_footprint frame.

This bypasses AMCL's internal TF lookup which fails in lifecycle nodes
due to a known ROS2 Humble lifecycle+TF2 issue.

Subscribe to the raw scan, look up the static transform chain
laser_link -> base_footprint (which works fine in a regular non-lifecycle
node), transform the scan, and republish on a new topic that AMCL can
consume without needing any TF lookup.
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


class ScanTransformer(Node):
    def __init__(self):
        super().__init__("scan_transformer")

        self.declare_parameter("base_frame", "robot1_base_footprint")
        self.declare_parameter("scan_in", "scan")
        self.declare_parameter("scan_out", "scan_transformed")

        base_frame = self.get_parameter("base_frame").value
        scan_in = self.get_parameter("scan_in").value
        scan_out = self.get_parameter("scan_out").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(LaserScan, scan_out, 10)
        self.sub = self.create_subscription(
            LaserScan, scan_in, self._callback, 10
        )

        self.get_logger().info(
            f"Transforming {scan_in} -> {scan_out} "
            f"(target frame: {base_frame})"
        )
        self._base_frame = base_frame
        self._first_success = True

    def _callback(self, scan: LaserScan):
        try:
            # Use time(0) in the node's clock type (ROS_TIME under sim)
            # because laser_link -> base_footprint is a fully static chain.
            t = self.tf_buffer.lookup_transform(
                self._base_frame,
                scan.header.frame_id,
                Time(clock_type=self.get_clock().clock_type),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
        except TransformException as e:
            self.get_logger().warn(
                f"TF lookup failed: {self._base_frame} <- {scan.header.frame_id} "
                f"at t={scan.header.stamp.sec}.{scan.header.stamp.nanosec}: {e}",
                throttle_duration_sec=2.0,
            )
            return

        if self._first_success:
            self.get_logger().info(
                f"First scan transformed: {scan.header.frame_id} -> {self._base_frame}"
            )
            self._first_success = False

        # Build a new scan header in the target frame
        out = LaserScan()
        out.header.stamp = scan.header.stamp
        out.header.frame_id = self._base_frame
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = scan.ranges
        out.intensities = scan.intensities
        self.pub.publish(out)


def main():
    rclpy.init()
    node = ScanTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

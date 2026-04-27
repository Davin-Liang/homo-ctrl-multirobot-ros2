#!/usr/bin/env python3
"""Republish /tf_static transforms on /tf with current timestamps.

Works around a known ROS2 Humble lifecycle+TF2 issue where nav2_amcl's
internal tf2_ros::TransformListener dedicated thread does not properly
receive /tf_static transforms with stamp(0,0) when use_sim_time is true.

Strategy:
- KEEP_ALL + TRANSIENT_LOCAL QoS: every published message is stored by
  the middleware and replayed to late-joining subscribers (AMCL's TF
  listener). This ensures that even though AMCL's listener subscribes
  after sim time has already advanced past early scan timestamps, it
  still receives the full history of static transform publications.
- 20 Hz republish: balances transform density with middleware storage
  (1200 messages/min ≈ 1.2 MB/min).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


def _copy_transform(src):
    dst = TransformStamped()
    dst.header.stamp.sec = src.header.stamp.sec
    dst.header.stamp.nanosec = src.header.stamp.nanosec
    dst.header.frame_id = src.header.frame_id
    dst.child_frame_id = src.child_frame_id
    dst.transform.translation.x = src.transform.translation.x
    dst.transform.translation.y = src.transform.translation.y
    dst.transform.translation.z = src.transform.translation.z
    dst.transform.rotation.x = src.transform.rotation.x
    dst.transform.rotation.y = src.transform.rotation.y
    dst.transform.rotation.z = src.transform.rotation.z
    dst.transform.rotation.w = src.transform.rotation.w
    return dst


class TfStaticBridge(Node):
    def __init__(self):
        super().__init__("tf_static_bridge")

        # KEEP_ALL + TRANSIENT_LOCAL: store every published message for
        # late joiners, so AMCL's TF buffer can be backfilled when its
        # listener finally subscribes.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_ALL,
            depth=1,  # ignored for KEEP_ALL
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(TFMessage, "/tf", qos)
        self.sub = self.create_subscription(
            TFMessage, "/tf_static", self._callback, qos
        )
        self.timer = self.create_timer(0.05, self._republish)  # 20 Hz
        self._latest_msg = None
        self._count = 0

    def _callback(self, msg):
        self._latest_msg = msg
        self.get_logger().info(
            f"Received {len(msg.transforms)} static transforms on /tf_static"
        )

    def _republish(self):
        if self._latest_msg is None:
            return
        now = self.get_clock().now().to_msg()
        out = TFMessage()
        for t in self._latest_msg.transforms:
            copy = _copy_transform(t)
            copy.header.stamp = now
            out.transforms.append(copy)
        self.pub.publish(out)

        self._count += 1
        if self._count == 1 or self._count % 200 == 0:
            self.get_logger().info(
                f"Published {self._count} batches on /tf at 20 Hz "
                f"(KEEP_ALL + TRANSIENT_LOCAL)"
            )


def main():
    rclpy.init()
    node = TfStaticBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

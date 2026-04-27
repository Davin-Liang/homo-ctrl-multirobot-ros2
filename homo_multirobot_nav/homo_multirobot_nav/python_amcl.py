#!/usr/bin/env python3
"""Python AMCL replacement that avoids the nav2_amcl lifecycle+TF2 bug.

Uses a regular (non-lifecycle) node so tf2_ros works correctly.
Implements a particle filter with likelihood field measurement model.
"""

import math
import random
import numpy as np
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseArray,
    Pose,
    Point,
    Quaternion,
    TransformStamped,
    Vector3,
)
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_ros import TransformBroadcaster


def euler_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


def quat_to_euler(q: Tuple[float, float, float, float]) -> float:
    return math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))


class LikelihoodField:
    """Precomputed likelihood field from an occupancy grid map."""

    def __init__(self, map_msg: OccupancyGrid, max_dist: float = 2.0):
        self.width = map_msg.info.width
        self.height = map_msg.info.height
        self.resolution = map_msg.info.resolution
        self.origin_x = map_msg.info.origin.position.x
        self.origin_y = map_msg.info.origin.position.y
        self.max_dist_cells = int(max_dist / self.resolution)

        # Compute distance field from obstacles
        grid = np.array(map_msg.data, dtype=np.int8).reshape(self.height, self.width)
        occupied = (grid > 50)  # occupied cells

        # Euclidean distance transform
        self.dist_field = np.full((self.height, self.width), float(self.max_dist_cells))

        # Seed with obstacle cells
        ys, xs = np.where(occupied)
        for y, x in zip(ys, xs):
            self.dist_field[y, x] = 0.0

        # Simple two-pass distance transform (cityblock approximation, then scale)
        # First pass: top-left to bottom-right
        for y in range(self.height):
            for x in range(self.width):
                if y > 0:
                    self.dist_field[y, x] = min(self.dist_field[y, x],
                                                self.dist_field[y - 1, x] + 1)
                if x > 0:
                    self.dist_field[y, x] = min(self.dist_field[y, x],
                                                self.dist_field[y, x - 1] + 1)

        # Second pass: bottom-right to top-left
        for y in range(self.height - 1, -1, -1):
            for x in range(self.width - 1, -1, -1):
                if y < self.height - 1:
                    self.dist_field[y, x] = min(self.dist_field[y, x],
                                                self.dist_field[y + 1, x] + 1)
                if x < self.width - 1:
                    self.dist_field[y, x] = min(self.dist_field[y, x],
                                                self.dist_field[y, x + 1] + 1)

        # Convert cell distances to meters
        self.dist_field = self.dist_field.astype(np.float64) * self.resolution

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        mx = max(0, min(self.width - 1, mx))
        my = max(0, min(self.height - 1, my))
        return mx, my

    def get_distance(self, wx: float, wy: float) -> float:
        mx, my = self.world_to_grid(wx, wy)
        return float(self.dist_field[my, mx])


class ParticleFilter:
    def __init__(self,
                 num_particles: int = 1000,
                 init_x: float = 0.0, init_y: float = 0.0, init_yaw: float = 0.0,
                 init_xy_std: float = 0.5, init_yaw_std: float = 0.5):
        self.num_particles = num_particles
        # Particles: [x, y, yaw, weight]
        self.particles = np.zeros((num_particles, 4), dtype=np.float64)
        self.particles[:, 0] = np.random.normal(init_x, init_xy_std, num_particles)
        self.particles[:, 1] = np.random.normal(init_y, init_xy_std, num_particles)
        self.particles[:, 2] = np.random.normal(init_yaw, init_yaw_std, num_particles)
        self.particles[:, 3] = 1.0 / num_particles

        # Motion model noise
        self.alpha_rot1 = 0.1
        self.alpha_trans = 0.1
        self.alpha_rot2 = 0.1

        # Measurement model
        self.z_hit = 0.5
        self.z_rand = 0.5
        self.sigma_hit = 0.2
        self.max_likelihood_dist = 2.0

        self.last_odom: Optional[Tuple[float, float, float]] = None

    def motion_update(self, odom_x: float, odom_y: float, odom_yaw: float):
        if self.last_odom is None:
            self.last_odom = (odom_x, odom_y, odom_yaw)
            return

        dx = odom_x - self.last_odom[0]
        dy = odom_y - self.last_odom[1]
        dyaw = odom_yaw - self.last_odom[2]
        dyaw = math.atan2(math.sin(dyaw), math.cos(dyaw))

        trans = math.sqrt(dx * dx + dy * dy)
        if trans < 0.001 and abs(dyaw) < 0.002:
            return

        # Vectorized noise
        n = self.num_particles
        drot1_std = self.alpha_rot1 * abs(dyaw) + self.alpha_rot2 * trans
        dtrans_std = self.alpha_trans * trans + self.alpha_rot1 * abs(dyaw)
        drot1_noise = np.random.normal(0.0, max(drot1_std, 0.001), n)
        dtrans_noise = np.random.normal(0.0, max(dtrans_std, 0.001), n)

        drot1 = dyaw + drot1_noise
        dtrans = trans + dtrans_noise

        self.particles[:, 0] += dtrans * np.cos(self.particles[:, 2] + drot1)
        self.particles[:, 1] += dtrans * np.sin(self.particles[:, 2] + drot1)
        self.particles[:, 2] += drot1
        self.particles[:, 2] = np.arctan2(np.sin(self.particles[:, 2]),
                                          np.cos(self.particles[:, 2]))

        self.last_odom = (odom_x, odom_y, odom_yaw)

    def measurement_update(self, scan: LaserScan, likelihood_field: LikelihoodField):
        z_max = scan.range_max
        n_beams = min(30, len(scan.ranges))
        beam_step = max(1, len(scan.ranges) // n_beams)
        beam_idx = list(range(0, len(scan.ranges), beam_step))[:n_beams]

        # Precompute beam angles and cos/sin
        angles = np.array([scan.angle_min + j * scan.angle_increment for j in beam_idx])
        ranges = np.array([scan.ranges[j] for j in beam_idx], dtype=np.float64)
        valid = (ranges > scan.range_min) & (ranges < z_max)
        beam_cos = np.cos(angles)
        beam_sin = np.sin(angles)

        p_rand = 1.0 / z_max
        sigma_hit = self.sigma_hit
        z_hit = self.z_hit
        z_rand = self.z_rand
        n = self.num_particles

        px = self.particles[:, 0]
        py = self.particles[:, 1]
        pyaw = self.particles[:, 2]
        cos_yaw = np.cos(pyaw)
        sin_yaw = np.sin(pyaw)

        weights = np.ones(n, dtype=np.float64)

        for k in range(len(beam_idx)):
            if not valid[k]:
                continue
            r = ranges[k]
            bcos = beam_cos[k]
            bsin = beam_sin[k]

            # Global beam endpoint for all particles (vectorized)
            gx = px + r * (cos_yaw * bcos - sin_yaw * bsin)
            gy = py + r * (sin_yaw * bcos + cos_yaw * bsin)

            # Distance lookup (must be per-particle for grid lookup)
            for i in range(n):
                dist = likelihood_field.get_distance(float(gx[i]), float(gy[i]))
                p_hit = math.exp(-0.5 * (dist / sigma_hit) ** 2) / \
                        (sigma_hit * math.sqrt(2 * math.pi))
                p = z_hit * p_hit + z_rand * p_rand
                if p < 1e-100:
                    p = 1e-100
                weights[i] *= p

        self.particles[:, 3] = weights

        # Normalize
        total = np.sum(weights)
        if total > 0:
            self.particles[:, 3] /= total
        else:
            self.particles[:, 3] = 1.0 / n

    def resample(self):
        weights = self.particles[:, 3]
        n = self.num_particles
        new_particles = np.zeros_like(self.particles)

        # Low variance resampling
        r = random.random() / n
        c = weights[0]
        i = 0
        for m in range(n):
            u = r + m / n
            while u > c and i < n - 1:
                i += 1
                c += weights[i]
            new_particles[m, :3] = self.particles[i, :3]
            new_particles[m, 3] = 1.0 / n

        self.particles = new_particles

    def get_estimate(self) -> Tuple[float, float, float, float, float, float]:
        """Returns (x, y, yaw, xx_cov, yy_cov, yawyaw_cov)."""
        x = np.average(self.particles[:, 0], weights=self.particles[:, 3])
        y = np.average(self.particles[:, 1], weights=self.particles[:, 3])
        # Circular mean for yaw
        cos_yaw = np.average(np.cos(self.particles[:, 2]), weights=self.particles[:, 3])
        sin_yaw = np.average(np.sin(self.particles[:, 2]), weights=self.particles[:, 3])
        yaw = math.atan2(sin_yaw, cos_yaw)

        # Covariance
        dx = self.particles[:, 0] - x
        dy = self.particles[:, 1] - y
        dyaw = self.particles[:, 2] - yaw
        dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))
        w = self.particles[:, 3]
        xx = np.average(dx * dx, weights=w)
        yy = np.average(dy * dy, weights=w)
        yyaw = np.average(dyaw * dyaw, weights=w)

        return x, y, yaw, xx, yy, yyaw

    def set_pose(self, x: float, y: float, yaw: float,
                 xy_std: float = 0.5, yaw_std: float = 0.5):
        self.particles[:, 0] = np.random.normal(x, xy_std, self.num_particles)
        self.particles[:, 1] = np.random.normal(y, xy_std, self.num_particles)
        self.particles[:, 2] = np.random.normal(yaw, yaw_std, self.num_particles)
        self.particles[:, 3] = 1.0 / self.num_particles
        self.last_odom = None


class PythonAMCL(Node):
    def __init__(self):
        super().__init__("python_amcl")
        self.declare_parameter("odom_frame_id", "")
        self.declare_parameter("base_frame_id", "base_footprint")
        self.declare_parameter("global_frame_id", "map")
        self.declare_parameter("min_particles", 1000)
        self.declare_parameter("max_particles", 2000)
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("set_initial_pose", True)
        self.declare_parameter("update_min_d", 0.2)
        self.declare_parameter("update_min_a", 0.2)
        self.declare_parameter("laser_max_beams", 60)
        self.declare_parameter("sigma_hit", 0.2)
        self.declare_parameter("z_hit", 0.5)
        self.declare_parameter("z_rand", 0.5)

        odom_frame = self.get_parameter("odom_frame_id").value
        base_frame = self.get_parameter("base_frame_id").value
        global_frame = self.get_parameter("global_frame_id").value
        scan_topic = self.get_parameter("scan_topic").value
        min_p = self.get_parameter("min_particles").value
        upd_d = self.get_parameter("update_min_d").value
        upd_a = self.get_parameter("update_min_a").value

        # Read initial pose — support both flat params and nav2-style nested dict
        init_x = 0.0; init_y = 0.0; init_yaw = 0.0
        try:
            ip = self.get_parameter("initial_pose").value
            if isinstance(ip, dict):
                init_x = float(ip.get("x", 0.0))
                init_y = float(ip.get("y", 0.0))
                init_yaw = float(ip.get("yaw", 0.0))
        except Exception:
            pass
        # Flat overrides
        for axis, param in [("x", "initial_pose_x"), ("y", "initial_pose_y"),
                            ("yaw", "initial_pose_yaw")]:
            try:
                v = self.get_parameter(param).value
                if v is not None:
                    if axis == "x": init_x = float(v)
                    elif axis == "y": init_y = float(v)
                    else: init_yaw = float(v)
            except Exception:
                pass

        self._base_frame = base_frame
        self._odom_frame = odom_frame if odom_frame else base_frame
        self._global_frame = global_frame
        self._update_min_d = upd_d
        self._update_min_a = upd_a

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Particle filter
        self.pf = ParticleFilter(
            num_particles=min_p,
            init_x=init_x, init_y=init_y, init_yaw=init_yaw)

        # Map / likelihood field
        self.likelihood_field: Optional[LikelihoodField] = None
        self.map_received = False

        # QoS for map: TRANSIENT_LOCAL to get late-joiner data
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, map_qos)

        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, 10)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "amcl_pose", 10)

        self.particlecloud_pub = self.create_publisher(
            PoseArray, "particlecloud", 10)

        self.init_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self._init_pose_cb, 10)

        self._last_estimate: Optional[Tuple[float, float, float]] = None
        self._scan_count = 0
        self._update_scan_skip = 3  # process every 3rd scan (~3.3 Hz)

        self.get_logger().info(
            f"Python AMCL started (base={self._base_frame}, global={self._global_frame}, "
            f"scan={scan_topic})")

    def _map_cb(self, msg: OccupancyGrid):
        self.likelihood_field = LikelihoodField(msg)
        self.map_received = True
        self.get_logger().info(
            f"Map received: {msg.info.width}x{msg.info.height} "
            f"@ {msg.info.resolution:.3f}m/px")

    def _init_pose_cb(self, msg: PoseWithCovarianceStamped):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_euler((q.x, q.y, q.z, q.w))
        self.pf.set_pose(x, y, yaw)
        self._last_estimate = None
        self.get_logger().info(f"Initial pose set to ({x:.2f}, {y:.2f}, {yaw:.2f})")

    def _get_odom(self, stamp: Time) -> Optional[Tuple[float, float, float]]:
        try:
            # odom -> base_footprint: robot pose in odometry frame
            t = self.tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                Time(clock_type=self.get_clock().clock_type),
                timeout=Duration(seconds=0.5),
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            yaw = quat_to_euler((q.x, q.y, q.z, q.w))
            return (x, y, yaw)
        except TransformException as e:
            self.get_logger().debug(f"Odom TF lookup failed: {e}")
            return None

    def _scan_cb(self, scan: LaserScan):
        if not self.map_received or self.likelihood_field is None:
            return

        self._scan_count += 1
        if self._scan_count % self._update_scan_skip != 0:
            return

        odom = self._get_odom(scan.header.stamp)
        if odom is None:
            return

        # Motion update
        self.pf.motion_update(odom[0], odom[1], odom[2])

        # Measurement update
        self.pf.measurement_update(scan, self.likelihood_field)

        # Resample
        self.pf.resample()

        # Get estimate
        x, y, yaw, xx, yy, yyaw = self.pf.get_estimate()

        # Publish pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = scan.header.stamp
        pose_msg.header.frame_id = self._global_frame
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0
        qx, qy, qz, qw = euler_to_quat(yaw)
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        # Fill covariance (6x6 upper triangular, row-major)
        cov = [0.0] * 36
        cov[0] = xx
        cov[7] = yy
        cov[35] = yyaw
        # xy cross-term
        dx = self.pf.particles[:, 0] - x
        dy = self.pf.particles[:, 1] - y
        w = self.pf.particles[:, 3]
        cov[1] = cov[6] = np.average(dx * dy, weights=w)
        pose_msg.pose.covariance = cov
        self.pose_pub.publish(pose_msg)

        # Publish particle cloud
        cloud = PoseArray()
        cloud.header.stamp = scan.header.stamp
        cloud.header.frame_id = self._global_frame
        step = max(1, self.pf.num_particles // 500)  # limit to ~500 for RViz
        for i in range(0, self.pf.num_particles, step):
            p = Pose()
            p.position.x = self.pf.particles[i, 0]
            p.position.y = self.pf.particles[i, 1]
            p.position.z = 0.0
            qx, qy, qz, qw = euler_to_quat(self.pf.particles[i, 2])
            p.orientation.x = qx
            p.orientation.y = qy
            p.orientation.z = qz
            p.orientation.w = qw
            cloud.poses.append(p)
        self.particlecloud_pub.publish(cloud)

        # Broadcast map -> odom transform
        # map_pose = odom_pose + (map -> odom)
        # map -> odom = map_pose - odom_pose (in map frame)
        odom_yaw = odom[2]
        cos_oy = math.cos(odom_yaw)
        sin_oy = math.sin(odom_yaw)
        # Transform robot pose from map frame to odom frame
        # T_map_robot = (x, y, yaw), T_map_odom = T_map_robot * inv(T_odom_robot)
        # T_odom_robot = (odom[0], odom[1], odom[2])
        # map->odom: rotation = yaw - odom_yaw, translation = [x,y] - R(map->odom)*[odom_x,odom_y]
        d_yaw = yaw - odom_yaw
        d_yaw = math.atan2(math.sin(d_yaw), math.cos(d_yaw))
        cos_dy = math.cos(d_yaw)
        sin_dy = math.sin(d_yaw)
        odom_in_map_x = cos_dy * odom[0] - sin_dy * odom[1]
        odom_in_map_y = sin_dy * odom[0] + cos_dy * odom[1]
        map_odom_x = x - odom_in_map_x
        map_odom_y = y - odom_in_map_y

        tf_msg = TransformStamped()
        tf_msg.header.stamp = scan.header.stamp
        tf_msg.header.frame_id = self._global_frame
        tf_msg.child_frame_id = self._odom_frame
        tf_msg.transform.translation.x = map_odom_x
        tf_msg.transform.translation.y = map_odom_y
        tf_msg.transform.translation.z = 0.0
        qx, qy, qz, qw = euler_to_quat(d_yaw)
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

        # Log first estimate
        if self._last_estimate is None:
            self.get_logger().info(
                f"First estimate: map=({x:.3f}, {y:.3f}, {yaw:.3f}), "
                f"odom=({odom[0]:.3f}, {odom[1]:.3f}, {odom[2]:.3f})")
        self._last_estimate = (x, y, yaw)


def main():
    rclpy.init()
    node = PythonAMCL()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

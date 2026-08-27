#!/usr/bin/env python3
"""Delay-aware closed-loop circular trajectory generator for a Leader robot.

The node holds a fixed map-frame heading while tracking a circular position
reference from EKF/odometry feedback.  It predicts the translational state over
the configured command dead time and first-order velocity-response horizon.
"""

from __future__ import annotations

from collections import deque
import math
from typing import Deque, Iterable, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
import tf2_ros


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """Return planar yaw from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def body_to_map(vector: np.ndarray, yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([c * vector[0] - s * vector[1],
                     s * vector[0] + c * vector[1]])


def map_to_body(vector: np.ndarray, yaw: float) -> np.ndarray:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([c * vector[0] + s * vector[1],
                     -s * vector[0] + c * vector[1]])


def odom_state_to_map(
    position_odom: np.ndarray,
    yaw_odom: float,
    velocity_body: np.ndarray,
    map_translation: np.ndarray,
    map_to_odom_yaw: float,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """Transform a planar odometry pose and body twist into the map frame."""
    position_map = map_translation + body_to_map(
        position_odom, map_to_odom_yaw)
    yaw_map = wrap_angle(map_to_odom_yaw + yaw_odom)
    velocity_map = body_to_map(velocity_body, yaw_map)
    return position_map, velocity_map, yaw_map


def circle_reference(
    p0: np.ndarray,
    radius: float,
    speed: float,
    omega: float,
    elapsed: float,
    start_side: str = 'top',
    phase_offset: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """Return a circle reference with p0 at the selected vertical endpoint."""
    if start_side not in ('top', 'bottom'):
        raise ValueError("start_side must be 'top' or 'bottom'")

    initial_phase = math.pi / 2.0 if start_side == 'top' else 3.0 * math.pi / 2.0
    phase = initial_phase + phase_offset + omega * elapsed
    center = p0 - radius * np.array([
        math.cos(initial_phase), math.sin(initial_phase)])
    position = center + radius * np.array([math.cos(phase), math.sin(phase)])
    direction_sign = 1.0 if omega >= 0.0 else -1.0
    velocity = direction_sign * speed * np.array([
        -math.sin(phase), math.cos(phase)])
    return position, velocity


def artstein_integral(
    command_history: Iterable[np.ndarray],
    dt: float,
    td: float,
    tau_v: float,
) -> np.ndarray:
    """Right-endpoint approximation of the 2-D first-order Artstein integral."""
    if td <= 0.0:
        return np.zeros(4)

    integral = np.zeros(4)
    segments = max(1, math.ceil(td / dt))
    for index, command in enumerate(command_history):
        if index >= segments:
            break
        weight = dt if index < segments - 1 else td - (segments - 1) * dt
        if weight <= 0.0:
            continue
        q = index * dt - td
        decay = math.exp(-q / tau_v)
        integral[0:2] += (1.0 - decay) * command * weight
        integral[2:4] += (decay / tau_v) * command * weight
    return integral


def predict_delayed_state(
    position: np.ndarray,
    velocity_map: np.ndarray,
    command_history: Iterable[np.ndarray],
    dt: float,
    td: float,
    tau_v: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Predict map-frame state at t + td + tau_v using final command history."""
    history = list(command_history)
    last_command = history[0] if history else np.zeros(2)
    z = np.r_[position, velocity_map] + artstein_integral(
        history, dt, td, tau_v)

    dead_time_decay = math.exp(-td / tau_v)
    delayed_velocity = dead_time_decay * z[2:4]
    delayed_position = z[0:2] + tau_v * (1.0 - dead_time_decay) * z[2:4]

    response_decay = math.exp(-1.0)
    predicted_velocity = last_command + response_decay * (
        delayed_velocity - last_command)
    predicted_position = delayed_position + last_command * tau_v + tau_v * (
        1.0 - response_decay) * (delayed_velocity - last_command)
    return predicted_position, predicted_velocity


def limit_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= maximum or norm <= 1e-12:
        return vector.copy()
    return vector * (maximum / norm)


def limit_delta(
    command: np.ndarray,
    previous: np.ndarray,
    max_rate: float,
    dt: float,
) -> np.ndarray:
    delta = command - previous
    maximum = max_rate * dt
    delta_norm = float(np.linalg.norm(delta))
    if delta_norm > maximum and delta_norm > 1e-12:
        delta *= maximum / delta_norm
    return previous + delta


class LeaderCircleClosedLoop(Node):
    def __init__(self) -> None:
        super().__init__('leader_circle_closed_loop')

        self.declare_parameter('radius', 2.0)
        self.declare_parameter('speed', 0.2)
        self.declare_parameter('heading', 0.0)
        self.declare_parameter('direction', 'ccw')
        self.declare_parameter('start_side', 'top')
        self.declare_parameter('rate', 20.0)
        self.declare_parameter('odom_topic', 'odometry/filtered')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('Td', 0.22)
        self.declare_parameter('tau_v', 0.43)
        self.declare_parameter('kp', 0.8)
        self.declare_parameter('kv', 0.2)
        self.declare_parameter('k_yaw', 1.5)
        self.declare_parameter('max_linear_vel', 0.4)
        self.declare_parameter('max_linear_accel', 0.25)
        self.declare_parameter('max_angular_vel', 0.8)
        self.declare_parameter('max_angular_accel', 1.0)

        self.radius = float(self.get_parameter('radius').value)
        self.speed = float(self.get_parameter('speed').value)
        self.heading = math.radians(float(self.get_parameter('heading').value))
        self.direction = str(self.get_parameter('direction').value)
        self.start_side = str(self.get_parameter('start_side').value)
        self.rate = float(self.get_parameter('rate').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.td = float(self.get_parameter('Td').value)
        self.tau_v = float(self.get_parameter('tau_v').value)
        self.kp = float(self.get_parameter('kp').value)
        self.kv = float(self.get_parameter('kv').value)
        self.k_yaw = float(self.get_parameter('k_yaw').value)
        self.max_linear_vel = float(self.get_parameter('max_linear_vel').value)
        self.max_linear_accel = float(
            self.get_parameter('max_linear_accel').value)
        self.max_angular_vel = float(
            self.get_parameter('max_angular_vel').value)
        self.max_angular_accel = float(
            self.get_parameter('max_angular_accel').value)

        if self.radius <= 0.0 or self.speed < 0.0 or self.rate <= 0.0:
            raise ValueError('radius and rate must be positive; speed must be non-negative')
        if self.td < 0.0 or self.tau_v <= 0.0:
            raise ValueError('Td must be non-negative and tau_v must be positive')
        if self.start_side not in ('top', 'bottom'):
            raise ValueError("start_side must be 'top' or 'bottom'")

        direction_sign = 1.0 if self.direction == 'ccw' else -1.0
        if self.direction not in ('ccw', 'cw'):
            self.get_logger().warn(
                f'unknown direction {self.direction!r}; using cw')
        self.omega_ref = direction_sign * self.speed / self.radius
        self.dt = 1.0 / self.rate
        history_length = max(1, math.ceil(self.td / self.dt)) + 2
        self.command_history: Deque[np.ndarray] = deque(maxlen=history_length)

        self.position: np.ndarray | None = None
        self.velocity_map: np.ndarray | None = None
        self.yaw: float | None = None
        self.p0: np.ndarray | None = None
        self.start_time: float | None = None
        self.frame_id = ''
        self.localization_valid = False
        self.tf_warning_emitted = False
        self.last_command_map = np.zeros(2)
        self.last_omega_command = 0.0

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        odom_topic = str(self.get_parameter('odom_topic').value)
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.get_logger().info(
            'closed-loop circle: '
            f'R={self.radius:.2f} m v={self.speed:.2f} m/s '
            f'omega={self.omega_ref:.3f} rad/s heading='
            f'{math.degrees(self.heading):.1f} deg '
            f'start_side={self.start_side} '
            f'map_frame={self.map_frame} '
            f'Td={self.td:.2f} s tau_v={self.tau_v:.2f} s')

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        yaw_odom = yaw_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w)
        body_velocity = np.array([
            message.twist.twist.linear.x,
            message.twist.twist.linear.y,
        ])
        position_odom = np.array([
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        ])
        odom_frame = message.header.frame_id

        try:
            if odom_frame == self.map_frame:
                map_translation = np.zeros(2)
                map_to_odom_yaw = 0.0
            else:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame, odom_frame, rclpy.time.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                map_translation = np.array([translation.x, translation.y])
                map_to_odom_yaw = yaw_from_quaternion(
                    rotation.x, rotation.y, rotation.z, rotation.w)
        except tf2_ros.TransformException as error:
            self.localization_valid = False
            if not self.tf_warning_emitted:
                self.get_logger().warn(
                    f'waiting for {self.map_frame} -> {odom_frame} TF: {error}')
                self.tf_warning_emitted = True
            return

        self.position, self.velocity_map, self.yaw = odom_state_to_map(
            position_odom, yaw_odom, body_velocity, map_translation,
            map_to_odom_yaw)
        self.localization_valid = True
        self.tf_warning_emitted = False

        if self.p0 is None:
            self.p0 = self.position.copy()
            self.start_time = self.now_seconds()
            self.frame_id = self.map_frame
            self.command_history.extend(
                [self.velocity_map.copy()] * self.command_history.maxlen)
            self.last_command_map = self.velocity_map.copy()
            self.get_logger().info(
                f'map reference initialized in frame {self.frame_id!r} at '
                f'({self.p0[0]:.3f}, {self.p0[1]:.3f})')

    def publish_zero(self) -> None:
        self.publisher.publish(Twist())

    def timer_callback(self) -> None:
        if (self.position is None or self.velocity_map is None or
                self.yaw is None or self.p0 is None or
                self.start_time is None or not self.localization_valid):
            self.publish_zero()
            return

        elapsed = max(0.0, self.now_seconds() - self.start_time)
        predicted_position, predicted_velocity = predict_delayed_state(
            self.position, self.velocity_map, self.command_history, self.dt,
            self.td, self.tau_v)
        lookahead = self.td + self.tau_v
        reference_position, reference_velocity = circle_reference(
            self.p0, self.radius, self.speed, self.omega_ref,
            elapsed + lookahead, self.start_side,
            phase_offset=-self.omega_ref * lookahead)

        map_command = reference_velocity - self.kp * (
            predicted_position - reference_position) - self.kv * (
            predicted_velocity - reference_velocity)
        map_command = limit_norm(map_command, self.max_linear_vel)
        map_command = limit_delta(
            map_command, self.last_command_map, self.max_linear_accel,
            self.dt)
        map_command = limit_norm(map_command, self.max_linear_vel)

        yaw_error = wrap_angle(self.heading - self.yaw)
        omega_command = max(
            -self.max_angular_vel,
            min(self.max_angular_vel, self.k_yaw * yaw_error))
        max_omega_delta = self.max_angular_accel * self.dt
        omega_command = max(
            self.last_omega_command - max_omega_delta,
            min(self.last_omega_command + max_omega_delta, omega_command))

        body_command = map_to_body(map_command, self.yaw)
        message = Twist()
        message.linear.x = float(body_command[0])
        message.linear.y = float(body_command[1])
        message.angular.z = float(omega_command)
        self.publisher.publish(message)

        final_map_command = body_to_map(body_command, self.yaw)
        self.last_command_map = final_map_command
        self.last_omega_command = omega_command
        self.command_history.appendleft(final_map_command.copy())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LeaderCircleClosedLoop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_zero()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

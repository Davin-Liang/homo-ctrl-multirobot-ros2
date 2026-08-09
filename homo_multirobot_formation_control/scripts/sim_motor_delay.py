#!/usr/bin/env python3
"""
仿真延迟模拟：放在控制器和 Gazebo 之间，模拟实物电机响应。

用法:
  # 默认: 低通滤波 100ms 时间常数，输出到原始 cmd_vel topic
  ros2 run homo_multirobot_formation_control sim_motor_delay.py \
    --ros-args -r __ns:=/robot2

  # 指定延迟参数
  ros2 run homo_multirobot_formation_control sim_motor_delay.py \
    --ros-args -r __ns:=/robot2 \
    -p input_topic:=/robot2/cmd_vel_raw \
    -p output_topic:=/robot2/cmd_vel \
    -p motor_tau:=0.15 -p transport_delay:=0.1 -p max_accel:=2.0

原理:
  cmd_vel 经低通滤波 + 加速度限幅后输出，模拟 STM32 速度环响应。
  配合 launch remap: controller → cmd_vel_raw → 本节点 → cmd_vel → Gazebo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from collections import deque


class SimMotorDelay(Node):
    def __init__(self):
        super().__init__('sim_motor_delay')

        self.declare_parameter('input_topic', 'cmd_vel')
        self.declare_parameter('output_topic', '')
        self.declare_parameter('motor_tau', 0.15)        # 低通滤波时间常数 (s)
        self.declare_parameter('transport_delay', 0.0)   # 纯传输延迟 (s)
        self.declare_parameter('max_accel', 2.0)         # 加速度限幅 (m/s²)
        self.declare_parameter('rate', 100.0)            # 内部控制频率 (Hz)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.motor_tau = self.get_parameter('motor_tau').value
        self.transport_delay = self.get_parameter('transport_delay').value
        self.max_accel = self.get_parameter('max_accel').value
        self.rate = self.get_parameter('rate').value

        if not self.output_topic:
            self.output_topic = 'cmd_vel_delayed'
            self.get_logger().warn(f'output_topic not set, using {self.output_topic}')

        self.sub = self.create_subscription(
            Twist, self.input_topic, self.cb, 10)
        self.pub = self.create_publisher(Twist, self.output_topic, 10)

        # 延迟缓冲: (target_time, twist_msg)
        self.delay_queue = deque()
        # 当前滤波后的速度
        self.vx_f = 0.0
        self.vy_f = 0.0
        self.omega_f = 0.0
        # 最新收到的目标速度
        self.latest_twist = Twist()
        self._tick = 0
        self._sum_queue = 0

        self.dt = 1.0 / self.rate
        self.timer = self.create_timer(self.dt, self.timer_cb)

        self.get_logger().info(
            f'SimMotorDelay: {self.input_topic} → [{self.motor_tau*1000:.0f}ms τ'
            f' + {self.transport_delay*1000:.0f}ms delay] → {self.output_topic}'
        )

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        deliver_at = now + self.transport_delay
        self.delay_queue.append((deliver_at, msg))

    def timer_cb(self):
        # 取出到期的延迟消息
        now = self.get_clock().now().nanoseconds / 1e9
        while self.delay_queue and self.delay_queue[0][0] <= now:
            _, self.latest_twist = self.delay_queue.popleft()

        target_vx = self.latest_twist.linear.x
        target_vy = self.latest_twist.linear.y
        target_omega = self.latest_twist.angular.z

        # 低通滤波: 一阶 IIR
        alpha = min(self.dt / (self.motor_tau + self.dt), 1.0)
        vx_raw = self.vx_f + alpha * (target_vx - self.vx_f)
        vy_raw = self.vy_f + alpha * (target_vy - self.vy_f)
        omega_raw = self.omega_f + alpha * (target_omega - self.omega_f)

        # 加速度限幅
        dv_max = self.max_accel * self.dt
        vx = clamp_rate(vx_raw, self.vx_f, dv_max)
        vy = clamp_rate(vy_raw, self.vy_f, dv_max)
        omega = clamp_rate(omega_raw, self.omega_f, dv_max)

        self.vx_f = vx
        self.vy_f = vy
        self.omega_f = omega
        self._tick += 1
        self._sum_queue += len(self.delay_queue)
        if self._tick % int(self.rate) == 0:
            avg_queue = self._sum_queue / max(self._tick, 1)
            self.get_logger().info(
                f'DELAY_TRACE q={len(self.delay_queue)} avg_q={avg_queue:.1f} '
                f'target=({target_vx:+.3f},{target_vy:+.3f}) '
                f'out=({vx:+.3f},{vy:+.3f}) tau={self.motor_tau:.2f} Td={self.transport_delay:.2f} '
                f'max_accel={self.max_accel:.2f}'
            )

        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = omega
        self.pub.publish(msg)


def clamp_rate(new_val, prev_val, max_step):
    return max(prev_val - max_step, min(prev_val + max_step, new_val))


def main():
    rclpy.init()
    node = SimMotorDelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

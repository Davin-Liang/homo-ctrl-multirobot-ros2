#!/usr/bin/env python3
"""
记录两台车在 map 坐标系下的轨迹、画图并保存 CSV 数据。

使用:
  # 仿真 + 自定义标签
  ros2 run homo_multirobot_formation_control record_trajectory.py \
    --ros-args -p mode:=sim -p tag:=hpc_0.3m -p duration:=30.0

  # 实物 + 编队半径参考线
  ros2 run homo_multirobot_formation_control record_trajectory.py \
    --ros-args -p mode:=real -p tag:=4d_mass8_r2 \
    -p leader_ns:=/virtual_leader -p follower_ns:=/robot2 \
    -p radius:=2.0 -p duration:=30.0

输出:
  {out_dir}/{mode}/{tag}_{timestamp}/*.png         ← 独立检查图
  {out_dir}/{mode}/{tag}_{timestamp}/raw.csv       ← MATLAB 可用
  {out_dir}/{mode}/{tag}_{timestamp}/metadata.yaml ← 实验元数据
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters, ListParameters
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
import tf2_ros
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory
import os
import csv
import json
import time
import math
import yaml
from datetime import datetime

PARAMETER_VALUE_FIELDS = {
    1: 'bool_value',
    2: 'integer_value',
    3: 'double_value',
    4: 'string_value',
    5: 'byte_array_value',
    6: 'bool_array_value',
    7: 'integer_array_value',
    8: 'double_array_value',
    9: 'string_array_value',
}

VALID_STATE_SOURCES = ('ekf_tf', 'mocap')
PLOT_FILENAMES = (
    'trajectory.png', 'distance.png', 'map_velocity.png', 'speed.png',
    'x.png', 'y.png', 'yaw.png',
)

RECORDER_PARAMETER_DEFAULTS = {
    'leader_ns': '/robot1',
    'follower_ns': '/robot2',
    'duration': 30.0,
    'out_dir': '',
    'radius': 0.0,
    'mode': 'sim',
    'tag': '',
    'controller_node_name': 'formation_control_node',
    'trial_id': 'trial_01',
    'controller': '',
    'state_source': 'ekf_tf',
}


def load_recorder_parameters(path):
    """读取并校验 ROS 2 格式的轨迹记录器参数 YAML。"""
    try:
        with open(path, encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'无法读取轨迹记录器配置 {path}: {exc}') from exc

    if not isinstance(document, dict):
        raise ValueError(f'轨迹记录器配置 {path} 必须包含 /**/ros__parameters 映射')
    node_parameters = document.get('/**')
    if not isinstance(node_parameters, dict):
        raise ValueError(f'轨迹记录器配置 {path} 缺少 /**/ros__parameters 映射')
    parameters = node_parameters.get('ros__parameters')
    if not isinstance(parameters, dict):
        raise ValueError(f'轨迹记录器配置 {path} 缺少 /**/ros__parameters 映射')
    parameters = dict(parameters)
    parameters.setdefault('state_source', 'ekf_tf')
    if set(parameters) != set(RECORDER_PARAMETER_DEFAULTS):
        raise ValueError(f'轨迹记录器配置 {path} 的参数名必须与内置记录器参数一致')
    return parameters


def validate_state_source(value):
    """校验并返回轨迹状态来源。"""
    if value not in VALID_STATE_SOURCES:
        raise ValueError(
            f"state_source 必须是 {', '.join(VALID_STATE_SOURCES)}，当前为 {value!r}")
    return value


def yaw_from_quaternion(quaternion):
    """从四元数提取绕 z 轴的航向角。"""
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def body_velocity_to_map(vx_body, vy_body, yaw):
    """将车体坐标系的平面线速度转换至 map 坐标系。"""
    return (vx_body * math.cos(yaw) - vy_body * math.sin(yaw),
            vx_body * math.sin(yaw) + vy_body * math.cos(yaw))


def map_yaw_from_odom(odom_yaw, map_to_odom_yaw):
    """组合 map→odom 与 odom→base 的平面朝向，并归一化至 [-pi, pi]。"""
    yaw = odom_yaw + map_to_odom_yaw
    return math.atan2(math.sin(yaw), math.cos(yaw))


def unwrap_yaw_series(yaws):
    """展开 yaw 序列，消除跨越 +/-pi 时的绘图跳变。"""
    if not yaws:
        return []
    result = [yaws[0]]
    for yaw in yaws[1:]:
        delta = (yaw - result[-1] + math.pi) % (2.0 * math.pi) - math.pi
        result.append(result[-1] + delta)
    return result


def mocap_sample(pose, twist):
    """从动捕适配器的 map 系消息中提取平面状态。"""
    return (pose.pose.position.x, pose.pose.position.y,
            yaw_from_quaternion(pose.pose.orientation),
            twist.twist.linear.x, twist.twist.linear.y,
            twist.twist.angular.z)


def recording_topics(leader_ns, follower_ns, state_source):
    """返回写入元数据的实际状态话题。"""
    if state_source == 'mocap':
        return {
            'leader_topic': leader_ns + '/mocap/pose',
            'follower_topic': follower_ns + '/mocap/pose',
            'leader_twist_topic': leader_ns + '/mocap/twist',
            'follower_twist_topic': follower_ns + '/mocap/twist',
        }
    return {
        'leader_topic': leader_ns + '/odometry/filtered',
        'follower_topic': follower_ns + '/odometry/filtered',
    }


def velocity_frame_label(state_source):
    """返回当前记录线速度所在参考系的图表标签。"""
    return 'Map-frame'


def merge_parameter_overrides(defaults, overrides):
    """将显式 ROS 参数覆盖到 YAML 默认值。"""
    values = dict(defaults)
    values.update({name: value for name, value in overrides.items() if name in values})
    return values


def parameter_value_to_python(parameter_value):
    """将 ROS 参数值转换为 YAML 可写入的 Python 值。"""
    field = PARAMETER_VALUE_FIELDS.get(parameter_value.type)
    if field is None:
        return None
    value = getattr(parameter_value, field)
    return list(value) if parameter_value.type >= 5 else value


def controller_parameter_mapping(names, values):
    """按参数名配对并忽略未设置的 ROS 参数值。"""
    params = {}
    for name, value in zip(names, values):
        converted = parameter_value_to_python(value)
        if converted is not None:
            params[name] = converted
    return params


def wait_for_controller_service(client, service_name, logger):
    """等待控制器参数服务就绪，ROS 关闭时停止等待。"""
    while rclpy.ok():
        if client.wait_for_service(timeout_sec=1.0):
            return True
        logger.info(f'等待控制器参数服务就绪: {service_name}')
    return False


def wait_for_controller_parameters(node, client, service_name, logger, names):
    """等待控制器参数响应，ROS 关闭时停止等待。"""
    while rclpy.ok():
        future = None
        try:
            req = GetParameters.Request()
            req.names = list(names)
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                return future.result()
        except Exception as exc:
            if future is not None and not future.done():
                future.cancel()
            logger.info(f'等待控制器参数响应: {service_name} ({exc})')
        else:
            if future is not None and not future.done():
                future.cancel()
            logger.info(f'等待控制器参数响应: {service_name}')

        if not rclpy.ok() or not wait_for_controller_service(client, service_name, logger):
            return None
    return None


def wait_for_controller_parameter_names(node, client, service_name, logger):
    """等待并读取控制器公开的全部参数名称。"""
    while rclpy.ok():
        future = None
        try:
            req = ListParameters.Request()
            req.depth = 0
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
            if future.done() and future.result() is not None:
                return sorted(set(future.result().result.names))
        except Exception as exc:
            if future is not None and not future.done():
                future.cancel()
            logger.info(f'等待控制器参数名称响应: {service_name} ({exc})')
        else:
            if future is not None and not future.done():
                future.cancel()
            logger.info(f'等待控制器参数名称响应: {service_name}')

        if not rclpy.ok() or not wait_for_controller_service(client, service_name, logger):
            return None
    return None


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        if not config_file:
            config_file = os.path.join(
                get_package_share_directory('homo_multirobot_formation_control'),
                'config', 'record_trajectory.yaml')
        try:
            yaml_defaults = load_recorder_parameters(config_file)
        except ValueError as exc:
            self.get_logger().fatal(str(exc))
            raise

        overrides = {
            name: parameter.value
            for name, parameter in self._parameter_overrides.items()
            if name in RECORDER_PARAMETER_DEFAULTS
        }
        recorder_parameters = merge_parameter_overrides(yaml_defaults, overrides)
        for name, value in recorder_parameters.items():
            self.declare_parameter(name, value, ignore_override=True)

        self.leader_ns = self.get_parameter('leader_ns').value
        self.follower_ns = self.get_parameter('follower_ns').value
        self.duration = self.get_parameter('duration').value
        self.out_dir = self.get_parameter('out_dir').value
        if not self.out_dir:
            pkg_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
            self.out_dir = os.path.join(pkg_dir, 'robot_traj')
        self.ideal_radius = self.get_parameter('radius').value
        self.mode = self.get_parameter('mode').value
        self.tag = self.get_parameter('tag').value
        self.ctrl_node_name = self.get_parameter('controller_node_name').value
        self.trial_id = self.get_parameter('trial_id').value
        self.controller = self.get_parameter('controller').value
        self.state_source = validate_state_source(
            self.get_parameter('state_source').value)

        # 查询控制器参数 + 延迟节点参数（自动生成 tag 和图上标题）
        self.ctrl_params = self._query_controller_params()
        if not rclpy.ok():
            raise KeyboardInterrupt
        self.delay_params = self._query_delay_node_params()
        if not self.tag:
            self.tag = self._build_auto_tag()
        # 输出到 {out_dir}/{mode}/ 子目录
        out_subdir = os.path.join(self.out_dir, self.mode)
        os.makedirs(out_subdir, exist_ok=True)

        self.t1_x = []; self.t1_y = []; self.t1_t = []
        self.t1_vx = []; self.t1_vy = []; self.t1_v = []
        self.t1_yaw = []; self.t1_omega = []
        self.t2_x = []; self.t2_y = []; self.t2_t = []
        self.t2_vx = []; self.t2_vy = []; self.t2_v = []
        self.t2_yaw = []; self.t2_omega = []

        self.t0 = None
        self.done = False

        if self.state_source == 'ekf_tf':
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self.sub1 = self.create_subscription(
                Odometry, self.leader_ns + '/odometry/filtered', self.cb_leader, 10)
            self.sub2 = self.create_subscription(
                Odometry, self.follower_ns + '/odometry/filtered', self.cb_follower, 10)
        else:
            self.leader_mocap_pose = None
            self.leader_mocap_twist = None
            self.follower_mocap_pose = None
            self.follower_mocap_twist = None
            self._create_mocap_subscriptions()
        self.timer = self.create_timer(0.1, self.check_done)

        leader_short = self.leader_ns.lstrip('/')
        follower_short = self.follower_ns.lstrip('/')
        self.leader_label = f'Leader ({leader_short})'
        self.follower_label = f'Follower ({follower_short})'
        self.get_logger().info(
            f'记录中... leader={self.leader_ns} follower={self.follower_ns} '
            f'时长={self.duration:.0f}s 模式={self.mode} 状态源={self.state_source} 标签={self.tag or "无"}'
            + (f' 理想半径={self.ideal_radius:.1f}m' if self.ideal_radius > 0 else ''))

    def _create_mocap_subscriptions(self):
        """创建与动捕适配器 SensorData QoS 匹配的状态订阅。"""
        self.leader_pose_sub = self.create_subscription(
            PoseStamped, self.leader_ns + '/mocap/pose', self.cb_leader_mocap_pose,
            qos_profile_sensor_data)
        self.leader_twist_sub = self.create_subscription(
            TwistStamped, self.leader_ns + '/mocap/twist', self.cb_leader_mocap_twist,
            qos_profile_sensor_data)
        self.follower_pose_sub = self.create_subscription(
            PoseStamped, self.follower_ns + '/mocap/pose', self.cb_follower_mocap_pose,
            qos_profile_sensor_data)
        self.follower_twist_sub = self.create_subscription(
            TwistStamped, self.follower_ns + '/mocap/twist', self.cb_follower_mocap_twist,
            qos_profile_sensor_data)

    def _query_controller_params(self):
        """从 follower 命名空间下的控制器节点读取参数。"""
        if not self.ctrl_node_name:
            self.get_logger().info('未设置 controller_node_name，跳过控制器参数查询')
            return {}

        node_path = self.follower_ns.rstrip('/') + '/' + self.ctrl_node_name
        list_svc_name = node_path + '/list_parameters'
        list_client = self.create_client(ListParameters, list_svc_name)

        if not wait_for_controller_service(
                list_client, list_svc_name, self.get_logger()):
            return {}

        names = wait_for_controller_parameter_names(
            self, list_client, list_svc_name, self.get_logger())
        if not names:
            return {}

        svc_name = node_path + '/get_parameters'
        client = self.create_client(GetParameters, svc_name)
        if not wait_for_controller_service(client, svc_name, self.get_logger()):
            return {}

        result = wait_for_controller_parameters(
            self, client, svc_name, self.get_logger(), names)
        if result is not None:
            params = controller_parameter_mapping(names, result.values)
            if params:
                self.get_logger().info(f'已读取控制器参数: {params}')
            return params
        return {}

    def _query_delay_node_params(self):
        """从 follower 命名空间下的 sim_motor_delay 节点读取参数（可选）。"""
        node_path = self.follower_ns.rstrip('/') + '/sim_motor_delay'
        svc_name = node_path + '/get_parameters'
        client = self.create_client(GetParameters, svc_name)
        if not client.wait_for_service(timeout_sec=2.0):
            return {}
        req = GetParameters.Request()
        req.names = ['motor_tau', 'transport_delay', 'max_accel', 'rate']
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.5)
        if future.done() and future.result() is not None:
            params = {}
            for name, pv in zip(req.names, future.result().values):
                if pv.type in (2, 3):
                    params[name] = pv.double_value if pv.type == 3 else pv.integer_value
            if params:
                self.get_logger().info(f'已读取延迟节点参数: {params}')
            return params
        return {}

    def _build_auto_tag(self):
        """根据控制器参数自动生成文件名标签。"""
        p = self.ctrl_params
        if not p:
            return 'default'
        parts = []
        parts.append(f"m{self._v(p, 'mass')}")
        parts.append(f"r{self._v(p, 'radius')}")
        if 'omega_d' in p:
            parts.append(f"od{self._v(p, 'omega_d')}")
        parts.append(f"f{self._v(p, 'control_rate')}")
        if 'tau' in p:
            parts.append(f"tau{self._v(p, 'tau')}")
        if 'hpc_c_min' in p:
            parts.append(f"cmin{self._v(p, 'hpc_c_min')}")
        if 'initial_min_lambda' in p:
            parts.append(f"ilam{self._v(p, 'initial_min_lambda')}")
        if 'switch_min_lambda' in p:
            parts.append(f"slam{self._v(p, 'switch_min_lambda')}")
        if 'Td' in p:
            parts.append(f"Td{self._v(p, 'Td')}")
        # 仿真延迟参数 (存在才加)
        dp = self.delay_params
        if dp:
            if 'motor_tau' in dp:
                parts.append(f"mtau{dp['motor_tau']:.2f}")
            if 'transport_delay' in dp:
                parts.append(f"td{dp['transport_delay']:.2f}")
            if 'max_accel' in dp:
                parts.append(f"da{dp['max_accel']:.2f}")
        return '_'.join(parts)

    @staticmethod
    def _v(params, key):
        """格式化单个参数值（去掉无意义的小数位）。"""
        v = params.get(key, 0)
        if abs(v - round(v)) < 0.01:
            return str(int(round(v)))
        return f'{v:.1f}'

    def _odom_to_map(self, ns, msg):
        odom_frame = ns.lstrip('/') + '_odom'
        try:
            t = self.tf_buffer.lookup_transform(
                'map', odom_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        tf_x = t.transform.translation.x
        tf_y = t.transform.translation.y
        q = t.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        tf_yaw = math.atan2(siny, cosy)
        ekf_x = msg.pose.pose.position.x
        ekf_y = msg.pose.pose.position.y
        return (tf_x + ekf_x * math.cos(tf_yaw) - ekf_y * math.sin(tf_yaw),
                tf_y + ekf_x * math.sin(tf_yaw) + ekf_y * math.cos(tf_yaw),
                tf_yaw)

    def _record(self, msg, ns, xl, yl, tl, vxl, vyl, vl, yawl, omegal):
        if self.done:
            return
        pos = self._odom_to_map(ns, msg)
        if pos is None:
            return
        now = time.time()
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('收到第一条数据, 开始计时')
        tl.append(now - self.t0)
        xl.append(pos[0])
        yl.append(pos[1])
        yaw = map_yaw_from_odom(
            yaw_from_quaternion(msg.pose.pose.orientation), pos[2])
        vx_map, vy_map = body_velocity_to_map(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, yaw)
        vxl.append(vx_map)
        vyl.append(vy_map)
        vl.append(math.hypot(vx_map, vy_map))
        yawl.append(yaw)
        omegal.append(msg.twist.twist.angular.z)

    def _record_mocap(self, pose, twist, xl, yl, tl, vxl, vyl, vl, yawl, omegal):
        if self.done:
            return
        x, y, yaw, vx, vy, omega = mocap_sample(pose, twist)
        now = time.time()
        if self.t0 is None:
            self.t0 = now
            self.get_logger().info('收到第一组完整状态, 开始计时')
        tl.append(now - self.t0)
        xl.append(x)
        yl.append(y)
        vxl.append(vx)
        vyl.append(vy)
        vl.append(math.hypot(vx, vy))
        yawl.append(yaw)
        omegal.append(omega)

    def cb_leader(self, msg):
        self._record(msg, self.leader_ns, self.t1_x, self.t1_y, self.t1_t,
                     self.t1_vx, self.t1_vy, self.t1_v, self.t1_yaw, self.t1_omega)
    def cb_follower(self, msg):
        self._record(msg, self.follower_ns, self.t2_x, self.t2_y, self.t2_t,
                     self.t2_vx, self.t2_vy, self.t2_v, self.t2_yaw, self.t2_omega)

    def _mocap_ready(self):
        return all((self.leader_mocap_pose, self.leader_mocap_twist,
                    self.follower_mocap_pose, self.follower_mocap_twist))

    def cb_leader_mocap_pose(self, msg):
        self.leader_mocap_pose = msg
        if self._mocap_ready():
            self._record_mocap(
                msg, self.leader_mocap_twist, self.t1_x, self.t1_y, self.t1_t,
                self.t1_vx, self.t1_vy, self.t1_v, self.t1_yaw, self.t1_omega)

    def cb_leader_mocap_twist(self, msg):
        self.leader_mocap_twist = msg

    def cb_follower_mocap_pose(self, msg):
        self.follower_mocap_pose = msg
        if self._mocap_ready():
            self._record_mocap(
                msg, self.follower_mocap_twist, self.t2_x, self.t2_y, self.t2_t,
                self.t2_vx, self.t2_vy, self.t2_v, self.t2_yaw, self.t2_omega)

    def cb_follower_mocap_twist(self, msg):
        self.follower_mocap_twist = msg

    def check_done(self):
        if self.done or self.t0 is None:
            return
        if time.time() - self.t0 >= self.duration:
            self.done = True
            self._save_and_plot()

    def _build_experiment_dir(self):
        """创建本次运行的独立实验目录。"""
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        dirname = f'{self.tag}_{ts}' if self.tag else ts
        base_dir = os.path.join(self.out_dir, self.mode, dirname)
        experiment_dir = base_dir
        suffix = 1
        while os.path.exists(experiment_dir):
            experiment_dir = f'{base_dir}_{suffix:02d}'
            suffix += 1
        os.makedirs(experiment_dir)
        return experiment_dir

    def _save_csv(self, experiment_dir):
        """保存对齐后的 CSV 数据（MATLAB 可直接 readtable）"""
        # 用 follower 的时间为基准，找最接近的 leader 数据点
        csv_path = os.path.join(experiment_dir, 'raw.csv')

        with open(csv_path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['time_s', 'leader_x_m', 'leader_y_m',
                        'leader_vx_map_ms', 'leader_vy_map_ms', 'leader_v_ms',
                        'leader_yaw_rad', 'leader_omega_rads',
                        'follower_x_m', 'follower_y_m',
                        'follower_vx_map_ms', 'follower_vy_map_ms', 'follower_v_ms',
                        'follower_yaw_rad', 'follower_omega_rads',
                        'distance_m'])
            n2 = len(self.t2_t)
            n1 = len(self.t1_t)
            for i2 in range(n2):
                t = self.t2_t[i2]
                fx = self.t2_x[i2]
                fy = self.t2_y[i2]
                fvx = self.t2_vx[i2]
                fvy = self.t2_vy[i2]
                fyaw = self.t2_yaw[i2]
                fomega = self.t2_omega[i2]
                # 找最接近的 leader 点
                i1 = min(range(n1), key=lambda j: abs(self.t1_t[j] - t))
                lx = self.t1_x[i1]
                ly = self.t1_y[i1]
                lvx = self.t1_vx[i1]
                lvy = self.t1_vy[i1]
                lyaw = self.t1_yaw[i1]
                lomega = self.t1_omega[i1]
                dist = math.hypot(lx - fx, ly - fy)
                w.writerow([f'{t:.4f}', f'{lx:.4f}', f'{ly:.4f}',
                            f'{lvx:.4f}', f'{lvy:.4f}', f'{math.hypot(lvx,lvy):.4f}',
                            f'{lyaw:.4f}', f'{lomega:.4f}',
                            f'{fx:.4f}', f'{fy:.4f}',
                            f'{fvx:.4f}', f'{fvy:.4f}', f'{math.hypot(fvx,fvy):.4f}',
                            f'{fyaw:.4f}', f'{fomega:.4f}',
                            f'{dist:.4f}'])
        self.get_logger().info(f'CSV 已保存: {csv_path}')

    @staticmethod
    def _yaml_scalar(value):
        """生成安全的 YAML 标量。"""
        if value is None:
            return 'null'
        if isinstance(value, bool):
            return 'true' if value else 'false'
        if isinstance(value, (int, float)):
            if isinstance(value, float) and not math.isfinite(value):
                return 'null'
            return str(value)
        return json.dumps(str(value), ensure_ascii=False)

    def _write_yaml_mapping(self, stream, mapping, indent=0):
        """写入本脚本所需的有限层级 YAML 映射。"""
        prefix = ' ' * indent
        for key, value in mapping.items():
            if isinstance(value, dict):
                stream.write(f'{prefix}{key}:\n')
                self._write_yaml_mapping(stream, value, indent + 2)
            elif isinstance(value, list):
                stream.write(f'{prefix}{key}: {json.dumps(value, ensure_ascii=False)}\n')
            else:
                stream.write(f'{prefix}{key}: {self._yaml_scalar(value)}\n')

    def _save_metadata(self, experiment_dir):
        """保存本次实验的元数据。"""
        yaml_path = os.path.join(experiment_dir, 'metadata.yaml')
        metadata = {
            'schema_version': 1,
            'trial_id': self.trial_id,
            'mode': self.mode,
            'controller': self.controller or self.ctrl_node_name,
            'recording': {
                'duration_s': self.duration,
                'leader_ns': self.leader_ns,
                'follower_ns': self.follower_ns,
                'state_source': self.state_source,
                **recording_topics(
                    self.leader_ns, self.follower_ns, self.state_source),
                'coordinate_frame': 'map',
                'ideal_radius_m': self.ideal_radius,
            },
            'controller_parameters': dict(self.ctrl_params),
            'delay_parameters': dict(self.delay_params),
            # 当前记录器不读取控制器内部目标点状态，避免写入错误值。
            'target_index': None,
            'desired_follower_x': None,
            'desired_follower_y': None,
            'files': {
                'csv': 'raw.csv',
                'plots': list(PLOT_FILENAMES),
            },
        }
        with open(yaml_path, 'w', encoding='utf-8') as f:
            self._write_yaml_mapping(f, metadata)
        self.get_logger().info(f'元数据已保存: {yaml_path}')

    def _plot_xy_vel(self, ax, tl, vl, name, c):
        """画速度曲线"""
        if not tl:
            return
        ax.plot(tl, vl, linewidth=0.8, label=name, color=c)

    def _plot_and_save(self, experiment_dir):
        """分别保存轨迹、位置、速度与姿态检查图。"""
        elapsed = time.time() - self.t0 if self.t0 else 0
        paths = []

        def save_figure(filename, draw):
            fig, ax = plt.subplots(figsize=(8, 6))
            draw(ax)
            fig.tight_layout()
            path = os.path.join(experiment_dir, filename)
            fig.savefig(path, dpi=150)
            plt.close(fig)
            paths.append(path)

        def finish(ax, xlabel, ylabel, title, legend_size=7):
            ax.set_xlabel(xlabel)
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            handles, _ = ax.get_legend_handles_labels()
            if handles:
                ax.legend(fontsize=legend_size)
            ax.grid(True, alpha=0.3)

        def draw_trajectory(ax):
            for xl, yl, name, color in [
                (self.t1_x, self.t1_y, self.leader_label, 'tab:blue'),
                (self.t2_x, self.t2_y, self.follower_label, 'tab:orange'),
            ]:
                if not xl:
                    continue
                ax.plot(xl, yl, linewidth=0.8, label=name, color=color)
                ax.scatter(xl[0], yl[0], c=color, marker='o', s=60, zorder=5)
                ax.scatter(xl[-1], yl[-1], c=color, marker='s', s=60, zorder=5)
            finish(ax, 'X (m)', 'Y (m)', f'[{self.mode}] {self.tag} ({elapsed:.1f}s)')
            ax.set_aspect('equal')

        def draw_distance(ax):
            n = min(len(self.t1_x), len(self.t2_x))
            if n:
                distance = [math.hypot(self.t1_x[i] - self.t2_x[i],
                                       self.t1_y[i] - self.t2_y[i]) for i in range(n)]
                mean = sum(distance) / n
                std = math.sqrt(sum((value - mean) ** 2 for value in distance) / n)
                ax.plot(self.t2_t[:n], distance, linewidth=1.0, color='tab:red',
                        label=f'Leader-follower (mean={mean:.2f}m, std={std:.2f}m)')
            if self.ideal_radius > 0:
                ax.axhline(self.ideal_radius, color='gray', linestyle='--', linewidth=1.2,
                           label=f'Ideal radius = {self.ideal_radius:.1f}m')
            finish(ax, 'Time (s)', 'Distance (m)', 'Leader-follower distance')

        def draw_map_velocity(ax):
            for tl, values, name, color in [
                (self.t1_t, self.t1_vx, self.leader_label + ' Vx', 'tab:blue'),
                (self.t2_t, self.t2_vx, self.follower_label + ' Vx', 'tab:orange'),
                (self.t1_t, self.t1_vy, self.leader_label + ' Vy', 'deepskyblue'),
                (self.t2_t, self.t2_vy, self.follower_label + ' Vy', 'gold'),
                (self.t1_t, self.t1_omega, self.leader_label + ' omega', 'tab:purple'),
                (self.t2_t, self.t2_omega, self.follower_label + ' omega', 'tab:green'),
            ]:
                self._plot_xy_vel(ax, tl, values, name, color)
            finish(ax, 'Time (s)', 'Map velocity (m/s), omega (rad/s)',
                   'Map-frame Vx, Vy & omega', 6)

        def draw_speed(ax):
            for tl, values, name, color in [
                (self.t1_t, self.t1_v, self.leader_label, 'tab:blue'),
                (self.t2_t, self.t2_v, self.follower_label, 'tab:orange'),
            ]:
                self._plot_xy_vel(ax, tl, values, name, color)
            finish(ax, 'Time (s)', 'Map-frame |V| (m/s)', 'Map-frame |V|')

        def draw_position(axis, coordinate, title):
            for tl, values, name, color in [
                (self.t1_t, coordinate[0], self.leader_label, 'tab:blue'),
                (self.t2_t, coordinate[1], self.follower_label, 'tab:orange'),
            ]:
                if tl:
                    axis.plot(tl, values, linewidth=0.8, label=name, color=color)
            finish(axis, 'Time (s)', title[0] + ' (m)', title[1])

        def draw_yaw(ax):
            for tl, yaws, name, color in [
                (self.t1_t, self.t1_yaw, self.leader_label, 'tab:blue'),
                (self.t2_t, self.t2_yaw, self.follower_label, 'tab:orange'),
            ]:
                if tl:
                    ax.plot(tl, unwrap_yaw_series(yaws), linewidth=0.8,
                            label=name, color=color)
            finish(ax, 'Time (s)', 'Yaw (rad, unwrapped)', 'Yaw tracking')

        save_figure('trajectory.png', draw_trajectory)
        save_figure('distance.png', draw_distance)
        save_figure('map_velocity.png', draw_map_velocity)
        save_figure('speed.png', draw_speed)
        save_figure('x.png', lambda ax: draw_position(ax, (self.t1_x, self.t2_x), ('X', 'X over time')))
        save_figure('y.png', lambda ax: draw_position(ax, (self.t1_y, self.t2_y), ('Y', 'Y over time')))
        save_figure('yaw.png', draw_yaw)

        self.get_logger().info(
            f'PNG 已保存: {", ".join(paths)}  '
            f'({self.leader_label}={len(self.t1_x)}, {self.follower_label}={len(self.t2_x)})')

    def _save_and_plot(self):
        experiment_dir = self._build_experiment_dir()
        self._save_csv(experiment_dir)
        self._plot_and_save(experiment_dir)
        self._save_metadata(experiment_dir)
        rclpy.shutdown()


def cleanup_node(node):
    """清理已创建节点，并在仍有效时关闭 ROS 上下文。"""
    if node is not None:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def main():
    rclpy.init()
    node = None
    try:
        node = TrajectoryRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cleanup_node(node)


if __name__ == '__main__':
    main()

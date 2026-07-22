/// 8D Pade 死区增广编队控制节点入口。
///
/// 状态 [px, py, vx_cmd, vy_cmd, ωx, ωy, vx_real, vy_real]，
/// 死区 Pade(1,1) + 电机滞后显式建模，详见 doc/pade_deadtime_full.md。
///
/// 使用: ros2 launch homo_multirobot_formation_control \
///         formation_single_follower_8d_motor.launch.py \
///         leader_ns:=/virtual_leader follower_ns:=/robot2

#include <rclcpp/rclcpp.hpp>
#include "homo_multirobot_formation_control/formation_control_node_8d_motor.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FormationController8DMotor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

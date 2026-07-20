/// 6D 电机感知模型编队控制节点入口。
///
/// 节点运行在 follower 命名空间下，cmd_vel 自动解析为 /<follower_ns>/cmd_vel:
///   ros2 launch homo_multirobot_formation_control formation_single_follower_6d_motor.launch.py
///
/// 状态 [px, py, vx_cmd, vy_cmd, vx_real, vy_real]，
/// 执行器一阶滞后显式建模，详见 doc/6d_motor_model_design.md。

#include <rclcpp/rclcpp.hpp>
#include "homo_multirobot_formation_control/formation_control_node_6d_motor.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FormationController6DMotor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

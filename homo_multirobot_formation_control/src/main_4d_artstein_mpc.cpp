/// 4D Artstein-MPC 编队控制节点入口。
///
/// 节点运行在 follower 命名空间下，cmd_vel 自动解析为 /<follower_ns>/cmd_vel:
///   ros2 launch homo_multirobot_formation_control formation_single_follower_4d_artstein_mpc.launch.py
///
/// 状态 [px, py, vx_real, vy_real]，Artstein 约简消除输入死区 Td，
/// 详见 doc/artstein_reduction.md。

#include <rclcpp/rclcpp.hpp>
#include "homo_multirobot_formation_control/formation_control_node_4d_artstein_mpc.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FormationController4DArtsteinMpc>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

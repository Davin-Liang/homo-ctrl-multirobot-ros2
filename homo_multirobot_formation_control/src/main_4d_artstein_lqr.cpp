/// 4D Artstein-LQR 编队控制节点入口。

#include <rclcpp/rclcpp.hpp>

#include "homo_multirobot_formation_control/formation_control_node_4d_artstein_lqr.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FormationController4DArtsteinLqr>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

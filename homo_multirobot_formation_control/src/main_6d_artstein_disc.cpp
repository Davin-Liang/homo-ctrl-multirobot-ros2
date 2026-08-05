/// 6D Artstein Disc 编队控制节点入口。

#include <rclcpp/rclcpp.hpp>

#include "homo_multirobot_formation_control/formation_control_node_6d_artstein_disc.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FormationController6DArtsteinDisc>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

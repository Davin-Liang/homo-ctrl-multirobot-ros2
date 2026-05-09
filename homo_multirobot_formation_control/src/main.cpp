/// 编队控制节点入口。
///
/// 节点运行在 follower 命名空间下，cmd_vel 自动解析为 /<follower_ns>/cmd_vel:
///   ros2 launch homo_multirobot_formation_control formation_single_follower.launch.py
///
/// 位置来自 TF、速度来自 EKF odometry/filtered，
/// 详见 formation_control_node.cpp 的数据管线。

#include <rclcpp/rclcpp.hpp>
#include "homo_multirobot_formation_control/formation_control_node.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FormationController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

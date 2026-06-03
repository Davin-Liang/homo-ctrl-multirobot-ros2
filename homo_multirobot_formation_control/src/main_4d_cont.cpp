/// 4D 连续边界投影编队控制节点入口。
///
/// 运行在 follower 命名空间下，cmd_vel 自动解析为 /<follower_ns>/cmd_vel:
///   ros2 launch homo_multirobot_formation_control formation_single_follower_4d_cont.launch.py

#include <rclcpp/rclcpp.hpp>
#include "homo_multirobot_formation_control/formation_control_node_4d_cont.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationController4DCont>());
  rclcpp::shutdown();
  return 0;
}

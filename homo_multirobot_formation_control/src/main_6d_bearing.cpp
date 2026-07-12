#include <rclcpp/rclcpp.hpp>
#include "homo_multirobot_formation_control/formation_control_node_6d_bearing.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationController6DBearing>());
  rclcpp::shutdown();
  return 0;
}

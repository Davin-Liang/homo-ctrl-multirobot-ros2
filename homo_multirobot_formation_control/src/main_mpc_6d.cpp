#include <rclcpp/rclcpp.hpp>
#include "homo_multirobot_formation_control/formation_control_node_mpc_6d.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FormationControllerMpc6D>());
  rclcpp::shutdown();
  return 0;
}

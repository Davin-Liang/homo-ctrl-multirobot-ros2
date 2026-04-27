#include <memory>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class OccupancyGridRelay : public rclcpp::Node
{
public:
  OccupancyGridRelay() : Node("occupancy_grid_relay")
  {
    const auto input_topic = this->declare_parameter<std::string>("input_topic", "/map");
    const auto output_topic = this->declare_parameter<std::string>("output_topic", "map");

    // Subscriber QoS: accept both volatile and transient_local publishers.
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    // Publisher QoS: transient_local so late-joiners (e.g. map_saver) still receive the latest map.
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic, pub_qos);
    sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      input_topic, sub_qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { pub_->publish(*msg); });

    RCLCPP_INFO(
      this->get_logger(), "Relaying OccupancyGrid: '%s' -> '%s'", input_topic.c_str(),
      output_topic.c_str());
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyGridRelay>());
  rclcpp::shutdown();
  return 0;
}


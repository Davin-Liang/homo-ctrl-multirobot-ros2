#include <cmath>
#include <limits>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class MocapStateAdapter : public rclcpp::Node
{
public:
  MocapStateAdapter() : Node("mocap_state_adapter")
  {
    input_pose_topic_ = declare_parameter("input_pose_topic", "/vrpn/robot1/pose");
    input_twist_topic_ = declare_parameter("input_twist_topic", "/vrpn/robot1/twist");
    output_pose_topic_ = declare_parameter("output_pose_topic", "mocap/pose");
    output_twist_topic_ = declare_parameter("output_twist_topic", "mocap/twist");
    map_frame_ = declare_parameter("map_frame", "map");
    odom_frame_ = declare_parameter("odom_frame", "robot1_odom");
    base_frame_ = declare_parameter("base_frame", "robot1_base_footprint");
    world_x_ = declare_parameter("world_x", 0.0);
    world_y_ = declare_parameter("world_y", 0.0);
    world_yaw_ = declare_parameter("world_yaw", 0.0);
    rigid_to_base_x_ = declare_parameter("rigid_to_base_x", 0.0);
    rigid_to_base_y_ = declare_parameter("rigid_to_base_y", 0.0);
    rigid_to_base_yaw_ = declare_parameter("rigid_to_base_yaw", 0.0);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(output_pose_topic_, rclcpp::SensorDataQoS());
    twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(output_twist_topic_, rclcpp::SensorDataQoS());
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_pub_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    publish_static_odom_tf();

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      input_pose_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) { pose_cb(*msg); });
    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_twist_topic_, rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::TwistStamped::SharedPtr msg) { twist_cb(*msg); });
  }

private:
  static double yaw_from(const geometry_msgs::msg::Quaternion & q)
  {
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    return yaw;
  }

  static geometry_msgs::msg::Quaternion yaw_quaternion(double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(q);
  }

  void publish_static_odom_tf()
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = map_frame_;
    tf.child_frame_id = odom_frame_;
    tf.transform.rotation.w = 1.0;
    static_tf_pub_->sendTransform(tf);
  }

  void pose_cb(const geometry_msgs::msg::PoseStamped & msg)
  {
    const double raw_yaw = yaw_from(msg.pose.orientation);
    const double c = std::cos(world_yaw_);
    const double s = std::sin(world_yaw_);
    const double rigid_x = world_x_ + c * msg.pose.position.x - s * msg.pose.position.y;
    const double rigid_y = world_y_ + s * msg.pose.position.x + c * msg.pose.position.y;
    const double rigid_yaw = world_yaw_ + raw_yaw;
    const double bc = std::cos(rigid_yaw);
    const double bs = std::sin(rigid_yaw);
    const double base_x = rigid_x + bc * rigid_to_base_x_ - bs * rigid_to_base_y_;
    const double base_y = rigid_y + bs * rigid_to_base_x_ + bc * rigid_to_base_y_;
    const double base_yaw = rigid_yaw + rigid_to_base_yaw_;

    geometry_msgs::msg::PoseStamped out;
    out.header = msg.header;
    out.header.frame_id = map_frame_;
    out.pose.position.x = base_x;
    out.pose.position.y = base_y;
    out.pose.position.z = msg.pose.position.z;
    out.pose.orientation = yaw_quaternion(base_yaw);
    pose_pub_->publish(out);

    geometry_msgs::msg::TransformStamped tf;
    tf.header = out.header;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = base_x;
    tf.transform.translation.y = base_y;
    tf.transform.translation.z = out.pose.position.z;
    tf.transform.rotation = out.pose.orientation;
    tf_pub_->sendTransform(tf);
  }

  void twist_cb(const geometry_msgs::msg::TwistStamped & msg)
  {
    const double c = std::cos(world_yaw_);
    const double s = std::sin(world_yaw_);
    geometry_msgs::msg::TwistStamped out;
    out.header = msg.header;
    out.header.frame_id = map_frame_;
    out.twist.linear.x = c * msg.twist.linear.x - s * msg.twist.linear.y;
    out.twist.linear.y = s * msg.twist.linear.x + c * msg.twist.linear.y;
    out.twist.linear.z = msg.twist.linear.z;
    out.twist.angular = msg.twist.angular;
    twist_pub_->publish(out);
  }

  std::string input_pose_topic_, input_twist_topic_, output_pose_topic_, output_twist_topic_;
  std::string map_frame_, odom_frame_, base_frame_;
  double world_x_, world_y_, world_yaw_, rigid_to_base_x_, rigid_to_base_y_, rigid_to_base_yaw_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MocapStateAdapter>());
  rclcpp::shutdown();
  return 0;
}

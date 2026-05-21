#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <Eigen/Dense>
#include <vector>

namespace formation_control {

class ObstacleAvoider
{
public:
  struct Obstacle
  {
    Eigen::Vector2d position;   // body frame (m)
    double radius;              // bounding circle radius (m)
    Eigen::Vector2d velocity;   // body frame (m/s), estimated
    int id;                     // tracking id
    int lost_frames;            // consecutive frames unmatched
  };

  /// @param node  ROS2 node pointer (used for param declaration, subscription, logging)
  explicit ObstacleAvoider(rclcpp::Node* node);

  /// LaserScan callback — runs at scan rate (~15 Hz).
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  /// Core solve: given HPC desired velocity and current velocity, returns
  /// optimal velocity that balances formation tracking and obstacle avoidance.
  /// All vectors in follower body frame.
  /// @param v_hpc     desired velocity from HPC controller [vx, vy, w]
  /// @param v_current current follower velocity [vx, vy, w]
  /// @param dt        control period (s)
  Eigen::Vector3d solve(const Eigen::Vector3d& v_hpc,
                        const Eigen::Vector3d& v_current,
                        double dt);

  const std::vector<Obstacle>& obstacles() const { return obstacles_; }
  bool scan_received() const { return scan_received_; }

private:
  // ---------- laser processing ----------

  void filter_and_cluster(const sensor_msgs::msg::LaserScan& scan);
  void associate_and_estimate_velocity(double dt);

  // ---------- QP helpers ----------

  /// Smooth max(0, x):  0.5 * (x + sqrt(x^2 + eps^2))
  static double penalty(double x);
  /// Derivative of smooth max(0, x)
  static double penalty_deriv(double x);

  /// Compute safe approach velocity and direction for one obstacle.
  void compute_safe_velocity(const Obstacle& obs,
                             double& v_safe, Eigen::Vector2d& n) const;

  /// Compute effective penalty weight for an obstacle.
  /// Scales up when robot is inside the safety zone.
  double obstacle_effective_weight(const Obstacle& obs) const;

  /// Evaluate objective J(v).
  double objective(const Eigen::Vector3d& v, const Eigen::Vector3d& v_hpc) const;

  /// Run one projected-gradient iteration. Returns true if converged.
  bool gradient_step(Eigen::Vector3d& v, const Eigen::Vector3d& v_hpc,
                     const Eigen::Vector3d& lb, const Eigen::Vector3d& ub);

  // ---------- parameters (declared in ctor) ----------

  double safety_distance_;
  double obstacle_weight_;
  double time_horizon_;
  int    max_obstacles_;
  double cluster_tolerance_;
  int    min_cluster_size_;
  double max_linear_vel_;
  double max_angular_vel_;
  double max_linear_accel_;
  double max_angular_accel_;

  // ---------- state ----------

  rclcpp::Node* node_;  // borrowed, not owning
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  bool scan_received_ = false;
  rclcpp::Time last_scan_stamp_;

  std::vector<Obstacle> obstacles_;
  std::vector<Obstacle> prev_obstacles_;
  int next_obstacle_id_ = 0;

  static constexpr double kSmoothEps   = 1e-4;  // smooth-max epsilon
  static constexpr double kScanTimeout = 0.5;   // clear obstacles if no scan for this long (s)
  static constexpr int    kMaxGradIters = 20;   // max gradient descent iterations
  static constexpr double kGradTol      = 1e-4; // convergence tolerance
};

}  // namespace formation_control

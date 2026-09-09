#pragma once

#include <algorithm>
#include <cmath>

namespace formation_control
{

inline double wrap_yaw_error(double error)
{
  return std::atan2(std::sin(error), std::cos(error));
}

inline double yaw_pd_command(double leader_yaw, double follower_yaw,
                             double leader_angular_z, double follower_angular_z,
                             double kp, double kd, double max_angular_vel)
{
  const double yaw_error = wrap_yaw_error(leader_yaw - follower_yaw);
  const double angular_velocity_error = leader_angular_z - follower_angular_z;
  return std::clamp(kp * yaw_error + kd * angular_velocity_error,
                    -max_angular_vel, max_angular_vel);
}

}  // namespace formation_control

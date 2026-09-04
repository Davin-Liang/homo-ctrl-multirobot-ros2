#pragma once

#include <array>
#include <string>
#include <vector>

namespace homo_multirobot_mocap_tools
{

struct ServerOptions
{
  int port{3883};
  std::string tracker_name{"robot1"};
  double radius{1.0};
  double speed{0.5};
  double rate{100.0};
};

struct CircleState
{
  std::array<double, 3> position{};
  std::array<double, 3> velocity{};
  std::array<double, 3> acceleration{};
  double yaw{};
  double angular_speed{};
};

bool parse_options(const std::vector<std::string> & args, ServerOptions & options, std::string & error);
CircleState circle_state(double seconds, const ServerOptions & options);

}  // namespace homo_multirobot_mocap_tools

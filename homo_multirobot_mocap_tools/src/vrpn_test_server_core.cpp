#include "homo_multirobot_mocap_tools/vrpn_test_server.hpp"

#include <cmath>
#include <stdexcept>

namespace homo_multirobot_mocap_tools
{
namespace
{

bool require_value(const std::vector<std::string> & args, std::size_t & index, std::string & value,
                   std::string & error)
{
  if (++index >= args.size()) {
    error = "missing value for " + args[index - 1];
    return false;
  }
  value = args[index];
  return true;
}

bool parse_int(const std::string & value, int & result)
{
  try {
    std::size_t parsed = 0;
    result = std::stoi(value, &parsed);
    return parsed == value.size();
  } catch (const std::exception &) {
    return false;
  }
}

bool parse_double(const std::string & value, double & result)
{
  try {
    std::size_t parsed = 0;
    result = std::stod(value, &parsed);
    return parsed == value.size() && std::isfinite(result);
  } catch (const std::exception &) {
    return false;
  }
}

}  // namespace

bool parse_options(const std::vector<std::string> & args, ServerOptions & options, std::string & error)
{
  options = ServerOptions{};
  error.clear();

  for (std::size_t index = 0; index < args.size(); ++index) {
    std::string value;
    if (args[index] == "--port") {
      if (!require_value(args, index, value, error) || !parse_int(value, options.port)) {
        error = error.empty() ? "invalid --port" : error;
        return false;
      }
    } else if (args[index] == "--tracker-name") {
      if (!require_value(args, index, options.tracker_name, error)) return false;
    } else if (args[index] == "--radius") {
      if (!require_value(args, index, value, error) || !parse_double(value, options.radius)) {
        error = error.empty() ? "invalid --radius" : error;
        return false;
      }
    } else if (args[index] == "--speed") {
      if (!require_value(args, index, value, error) || !parse_double(value, options.speed)) {
        error = error.empty() ? "invalid --speed" : error;
        return false;
      }
    } else if (args[index] == "--rate") {
      if (!require_value(args, index, value, error) || !parse_double(value, options.rate)) {
        error = error.empty() ? "invalid --rate" : error;
        return false;
      }
    } else {
      error = "unknown option " + args[index];
      return false;
    }
  }

  if (options.port < 1 || options.port > 65535 || options.tracker_name.empty() ||
    options.radius <= 0.0 || options.speed < 0.0 || options.rate <= 0.0)
  {
    error = "port must be 1..65535, tracker name must be non-empty, radius/rate must be positive, speed non-negative";
    return false;
  }
  return true;
}

CircleState circle_state(double seconds, const ServerOptions & options)
{
  const double omega = options.speed / options.radius;
  const double phase = omega * seconds;
  CircleState state;
  state.position = {options.radius * std::cos(phase), options.radius * std::sin(phase), 0.0};
  state.velocity = {-options.speed * std::sin(phase), options.speed * std::cos(phase), 0.0};
  state.acceleration = {
    -options.speed * omega * std::cos(phase), -options.speed * omega * std::sin(phase), 0.0};
  state.yaw = phase + std::acos(-1.0) / 2.0;
  state.angular_speed = omega;
  return state;
}

}  // namespace homo_multirobot_mocap_tools

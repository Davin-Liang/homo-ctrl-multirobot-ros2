#include "homo_multirobot_mocap_tools/vrpn_test_server.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <sys/time.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>

namespace
{

std::atomic_bool running{true};

void stop_server(int)
{
  running = false;
}

void print_usage(const char * executable)
{
  std::cout << "Usage: " << executable
            << " [--port PORT] [--tracker-name NAME] [--radius METERS]"
            << " [--speed MPS] [--rate HZ]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  std::vector<std::string> args(argv + 1, argv + argc);
  if (args.size() == 1 && (args.front() == "--help" || args.front() == "-h")) {
    print_usage(argv[0]);
    return 0;
  }

  homo_multirobot_mocap_tools::ServerOptions options;
  std::string error;
  if (!homo_multirobot_mocap_tools::parse_options(args, options, error)) {
    std::cerr << "Error: " << error << '\n';
    print_usage(argv[0]);
    return 2;
  }

  vrpn_Connection * connection = vrpn_create_server_connection(options.port);
  if (connection == nullptr) {
    std::cerr << "Error: unable to listen on VRPN TCP port " << options.port << '\n';
    return 1;
  }
  vrpn_Tracker_Server tracker(options.tracker_name.c_str(), connection, 1);

  std::signal(SIGINT, stop_server);
  std::signal(SIGTERM, stop_server);
  const auto started = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration<double>(1.0 / options.rate);
  auto next_tick = started;
  std::cout << "VRPN test server listening on 0.0.0.0:" << options.port
            << ", tracker=" << options.tracker_name
            << ", client address=" << options.tracker_name << "@<server-ip>:" << options.port << '\n';

  while (running) {
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const auto state = homo_multirobot_mocap_tools::circle_state(elapsed, options);
    const double pose_quat[4] = {0.0, 0.0, std::sin(state.yaw / 2.0), std::cos(state.yaw / 2.0)};
    const double angular_step = state.angular_speed / options.rate;
    const double velocity_quat[4] = {0.0, 0.0, std::sin(angular_step / 2.0), std::cos(angular_step / 2.0)};
    const double identity_quat[4] = {0.0, 0.0, 0.0, 1.0};
    timeval timestamp{};
    gettimeofday(&timestamp, nullptr);

    tracker.report_pose(0, timestamp, state.position.data(), pose_quat);
    tracker.report_pose_velocity(0, timestamp, state.velocity.data(), velocity_quat, 1.0 / options.rate);
    tracker.report_pose_acceleration(0, timestamp, state.acceleration.data(), identity_quat, 1.0 / options.rate);
    tracker.mainloop();
    connection->mainloop();

    next_tick += std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    std::this_thread::sleep_until(next_tick);
  }

  connection->removeReference();
  return 0;
}

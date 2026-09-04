#include <gtest/gtest.h>

#include <cmath>

#include "homo_multirobot_mocap_tools/vrpn_test_server.hpp"

namespace tools = homo_multirobot_mocap_tools;

TEST(VrpnTestServer, UsesDocumentedDefaults)
{
  tools::ServerOptions options;
  std::string error;

  EXPECT_TRUE(tools::parse_options({}, options, error));
  EXPECT_TRUE(error.empty());
  EXPECT_EQ(options.port, 3883);
  EXPECT_EQ(options.tracker_name, "robot1");
  EXPECT_DOUBLE_EQ(options.radius, 1.0);
  EXPECT_DOUBLE_EQ(options.speed, 0.5);
  EXPECT_DOUBLE_EQ(options.rate, 100.0);
}

TEST(VrpnTestServer, RejectsInvalidPort)
{
  tools::ServerOptions options;
  std::string error;

  EXPECT_FALSE(tools::parse_options({"--port", "0"}, options, error));
  EXPECT_FALSE(error.empty());
}

TEST(VrpnTestServer, RejectsZeroRate)
{
  tools::ServerOptions options;
  std::string error;

  EXPECT_FALSE(tools::parse_options({"--rate", "0"}, options, error));
  EXPECT_FALSE(error.empty());
}

TEST(VrpnTestServer, ComputesAnalyticCircleStateAtStart)
{
  tools::ServerOptions options;
  options.radius = 2.0;
  options.speed = 1.0;

  const auto state = tools::circle_state(0.0, options);

  EXPECT_NEAR(state.position[0], 2.0, 1e-12);
  EXPECT_NEAR(state.position[1], 0.0, 1e-12);
  EXPECT_NEAR(state.velocity[0], 0.0, 1e-12);
  EXPECT_NEAR(state.velocity[1], 1.0, 1e-12);
  EXPECT_NEAR(state.acceleration[0], -0.5, 1e-12);
  EXPECT_NEAR(state.acceleration[1], 0.0, 1e-12);
  EXPECT_NEAR(state.yaw, M_PI_2, 1e-12);
  EXPECT_NEAR(state.angular_speed, 0.5, 1e-12);
}

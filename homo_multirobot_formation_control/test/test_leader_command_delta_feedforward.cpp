#include <cassert>
#include <cmath>

#include <Eigen/Dense>

#include "homo_multirobot_formation_control/leader_command_delta_feedforward.hpp"

int main()
{
  formation_control::LeaderCommandDeltaFeedforward feedforward(true, 0.0, 0.6 / 20.0);
  feedforward.ingest(Eigen::Vector2d(0.2, 0.0), 1.0);
  assert(feedforward.consume(1.0, 0.15).isZero(1e-12));

  feedforward.ingest(Eigen::Vector2d(0.22, -0.01), 1.05);
  assert((feedforward.consume(1.05, 0.15) - Eigen::Vector2d(0.02, -0.01)).norm() < 1e-12);
  assert(feedforward.consume(1.06, 0.15).isZero(1e-12));

  formation_control::LeaderCommandDeltaFeedforward capped(true, 0.0, 0.6 / 20.0);
  capped.ingest(Eigen::Vector2d::Zero(), 1.0);
  capped.ingest(Eigen::Vector2d(0.3, 0.4), 1.05);
  assert(std::abs(capped.consume(1.05, 0.15).norm() - 0.03) < 1e-12);

  formation_control::LeaderCommandDeltaFeedforward expired(true, 0.0, 1.0);
  expired.ingest(Eigen::Vector2d::Zero(), 1.0);
  expired.ingest(Eigen::Vector2d(0.1, 0.0), 1.05);
  assert(expired.consume(2.0, 0.15).isZero(1e-12));

  formation_control::LeaderCommandDeltaFeedforward disabled(false, 0.0, 1.0);
  disabled.ingest(Eigen::Vector2d::Zero(), 1.0);
  disabled.ingest(Eigen::Vector2d(0.1, 0.0), 1.05);
  assert(disabled.consume(1.05, 0.15).isZero(1e-12));
  return 0;
}

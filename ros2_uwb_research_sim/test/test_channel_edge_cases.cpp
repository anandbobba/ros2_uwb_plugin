// Copyright 2026 Anand Bobba
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//
// Edge-case unit tests for the UWB channel and physics models.
// Supplements test_noise_models.cpp with boundary conditions:
//   - Zero distance (anchor == tag position)
//   - Very large distances (100 m)
//   - Extreme noise parameter values
//   - Measured range positivity under all conditions

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "ros2_uwb_research_sim/physics_model.hpp"
#include "ros2_uwb_research_sim/channel_model.hpp"
#include "ros2_uwb_research_sim/gaussian_noise_model.hpp"
#include "ros2_uwb_research_sim/nlos_bias_model.hpp"

using ros2_uwb_research_sim::ChannelModel;
using ros2_uwb_research_sim::ChannelParams;
using ros2_uwb_research_sim::GaussianNoiseModel;
using ros2_uwb_research_sim::NLOSBiasModel;
using ros2_uwb_research_sim::PhysicsModel;
using ros2_uwb_research_sim::UWBMeasurement;

// =============================================================================
// PhysicsModel edge cases
// =============================================================================

TEST(PhysicsModel, ZeroDistanceAnchorEqualsTag) {
  PhysicsModel m;
  Eigen::Vector3d pos(3.7, -2.1, 1.5);
  EXPECT_DOUBLE_EQ(m.computeDistance(pos, pos), 0.0);
}

TEST(PhysicsModel, VeryLargeDistance100m) {
  PhysicsModel m;
  // diagonal of a 100m cube: sqrt(3) * 100 ≈ 173.2 m
  double d = m.computeDistance(
    Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(100, 100, 100));
  EXPECT_NEAR(d, std::sqrt(3.0) * 100.0, 1e-6);
}

TEST(PhysicsModel, NegativeCoordinatesLargeDistance) {
  PhysicsModel m;
  double d = m.computeDistance(
    Eigen::Vector3d(-50, -50, 0),
    Eigen::Vector3d(50, 50, 0));
  EXPECT_NEAR(d, std::sqrt(2.0) * 100.0, 1e-6);
}

// =============================================================================
// ChannelModel — zero-noise, large distance
// =============================================================================

TEST(ChannelModel, LargeDistanceMeasurementIsAccurate) {
  ChannelParams p;
  p.gaussian_sigma = 0.0;
  p.nlos_lambda = 1.0;
  p.nlos_prob = 0.0;
  p.multipath_alpha = 0.0;
  p.multipath_sigma = 0.0;
  p.clock_drift_sigma = 0.0;

  ChannelModel m(p);
  Eigen::Vector3d anchor(0, 0, 0);
  Eigen::Vector3d tag(60, 80, 0);  // true = 100 m

  for (int i = 0; i < 20; ++i) {
    EXPECT_NEAR(m.computeMeasuredRange(anchor, tag), 100.0, 1e-9);
  }
}

TEST(ChannelModel, ZeroDistanceMeasurementNearZero) {
  // With no noise, measured range for co-located anchor/tag should be ~0
  ChannelParams p;
  p.gaussian_sigma = 0.0;
  p.nlos_lambda = 1.0;
  p.nlos_prob = 0.0;
  p.multipath_alpha = 0.0;
  p.multipath_sigma = 0.0;
  p.clock_drift_sigma = 0.0;

  ChannelModel m(p);
  Eigen::Vector3d pos(1.0, 2.0, 3.0);
  EXPECT_NEAR(m.computeMeasuredRange(pos, pos), 0.0, 1e-9);
}

// =============================================================================
// ChannelModel — measured range stays within reasonable bounds under noise
// =============================================================================

TEST(ChannelModel, HighGaussianNoiseSurvivesWithoutCrash) {
  ChannelParams p;
  p.gaussian_sigma = 10.0;  // Extreme: 10 m std dev
  p.nlos_lambda = 1.0;
  p.nlos_prob = 0.0;
  p.multipath_alpha = 0.0;
  p.multipath_sigma = 0.0;
  p.clock_drift_sigma = 0.0;

  ChannelModel m(p);
  Eigen::Vector3d anchor(0, 0, 0), tag(50, 0, 0);  // true = 50 m

  // Should not throw or produce NaN / Inf
  for (int i = 0; i < 100; ++i) {
    double r = m.computeMeasuredRange(anchor, tag);
    EXPECT_FALSE(std::isnan(r));
    EXPECT_FALSE(std::isinf(r));
  }
}

TEST(ChannelModel, AlwaysNLOSMeasurementIsAboveTrueDistance) {
  // When NLOS probability = 1 and no other noise, measured >= d_true always
  ChannelParams p;
  p.gaussian_sigma = 0.0;
  p.nlos_lambda = 2.0;
  p.nlos_prob = 1.0;
  p.multipath_alpha = 0.0;
  p.multipath_sigma = 0.0;
  p.clock_drift_sigma = 0.0;

  ChannelModel m(p);
  Eigen::Vector3d anchor(0, 0, 0), tag(20, 0, 0);  // true = 20 m

  for (int i = 0; i < 200; ++i) {
    UWBMeasurement meas = m.computeDetailedMeasurement(anchor, tag);
    EXPECT_GE(meas.measured, meas.d_true - 1e-9);
  }
}

TEST(ChannelModel, ComponentSumInvariantAtLargeDistance) {
  // measured = d_true + gaussian + nlos + multipath + drift (always)
  ChannelParams p;
  p.gaussian_sigma = 0.2;
  p.nlos_lambda = 1.5;
  p.nlos_prob = 0.2;
  p.multipath_alpha = 0.4;
  p.multipath_sigma = 0.1;
  p.clock_drift_sigma = 0.005;

  ChannelModel m(p);
  Eigen::Vector3d anchor(0, 0, 0), tag(80, 0, 0);  // 80 m

  for (int i = 0; i < 100; ++i) {
    UWBMeasurement meas = m.computeDetailedMeasurement(anchor, tag);
    double expected = meas.d_true + meas.gaussian_noise +
      meas.nlos_bias + meas.multipath_error + meas.clock_drift;
    EXPECT_NEAR(meas.measured, expected, 1e-9);
  }
}

TEST(ChannelModel, ExtremeParamsSurviveWithoutCrash) {
  // Stress test: all params at aggressive values
  ChannelParams p;
  p.gaussian_sigma = 5.0;
  p.nlos_lambda = 0.1;    // Very heavy tail
  p.nlos_prob = 1.0;      // Always NLOS
  p.multipath_alpha = 0.99;  // Near-unstable AR(1)
  p.multipath_sigma = 1.0;
  p.clock_drift_sigma = 0.1;

  ChannelModel m(p);
  Eigen::Vector3d anchor(0, 0, 0), tag(10, 0, 0);

  for (int i = 0; i < 50; ++i) {
    double r = m.computeMeasuredRange(anchor, tag);
    EXPECT_FALSE(std::isnan(r));
    EXPECT_FALSE(std::isinf(r));
  }
}

// =============================================================================
// GaussianNoiseModel — additional edge cases
// =============================================================================

TEST(GaussianNoiseModel, ExtremeSigmaProducesFiniteOutput) {
  GaussianNoiseModel m(100.0);
  for (int i = 0; i < 50; ++i) {
    double s = m.sample();
    EXPECT_FALSE(std::isnan(s));
    EXPECT_FALSE(std::isinf(s));
  }
}

// =============================================================================
// NLOSBiasModel — additional edge cases
// =============================================================================

TEST(NLOSBiasModel, VeryHighLambdaProducesSmallBias) {
  // lambda=100 → mean of Exp = 0.01 m
  NLOSBiasModel m(100.0, 1.0);
  double sum = 0.0;
  const int N = 1000;
  for (int i = 0; i < N; ++i) {
    sum += m.sample();
  }
  EXPECT_LT(sum / N, 0.1);  // mean << 0.1 m
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

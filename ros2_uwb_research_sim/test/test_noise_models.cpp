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
// Unit tests for UWB noise and channel models.
// Tests statistical properties and invariants without full ROS runtime.

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <cmath>

#include "ros2_uwb_research_sim/physics_model.hpp"
#include "ros2_uwb_research_sim/gaussian_noise_model.hpp"
#include "ros2_uwb_research_sim/nlos_bias_model.hpp"
#include "ros2_uwb_research_sim/multipath_model.hpp"
#include "ros2_uwb_research_sim/clock_drift_model.hpp"
#include "ros2_uwb_research_sim/channel_model.hpp"

using ros2_uwb_research_sim::ChannelModel;
using ros2_uwb_research_sim::ChannelParams;
using ros2_uwb_research_sim::ClockDriftModel;
using ros2_uwb_research_sim::GaussianNoiseModel;
using ros2_uwb_research_sim::MultipathModel;
using ros2_uwb_research_sim::NLOSBiasModel;
using ros2_uwb_research_sim::PhysicsModel;
using ros2_uwb_research_sim::UWBMeasurement;

// =============================================================================
// PhysicsModel
// =============================================================================

TEST(PhysicsModel, OriginToOriginIsZero) {
  PhysicsModel m;
  EXPECT_DOUBLE_EQ(m.computeDistance(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()), 0.0);
}

TEST(PhysicsModel, KnownDistance3_4_5) {
  PhysicsModel m;
  // 3-4-5 right triangle
  EXPECT_NEAR(m.computeDistance(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(4, 4, 0)), 5.0, 1e-9);
}

TEST(PhysicsModel, KnownDistance3D) {
  PhysicsModel m;
  // (1,2,2) has norm = 3
  EXPECT_NEAR(m.computeDistance(Eigen::Vector3d::Zero(), Eigen::Vector3d(1, 2, 2)), 3.0, 1e-9);
}

TEST(PhysicsModel, Symmetric) {
  PhysicsModel m;
  Eigen::Vector3d a(1, 2, 3), b(7, 5, 1);
  EXPECT_DOUBLE_EQ(m.computeDistance(a, b), m.computeDistance(b, a));
}

TEST(PhysicsModel, AlwaysNonNegative) {
  PhysicsModel m;
  EXPECT_GE(m.computeDistance(Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(2, 2, 2)), 0.0);
  EXPECT_GE(m.computeDistance(Eigen::Vector3d(-3, -3, 0), Eigen::Vector3d(3, 3, 0)), 0.0);
}

// =============================================================================
// GaussianNoiseModel
// =============================================================================

TEST(GaussianNoiseModel, ZeroSigmaAlwaysZero) {
  GaussianNoiseModel m(0.0);
  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(m.sample(), 0.0);
  }
}

TEST(GaussianNoiseModel, MeanNearZero) {
  GaussianNoiseModel m(1.0);
  double sum = 0.0;
  const int N = 10000;
  for (int i = 0; i < N; ++i) {
    sum += m.sample();
  }
  EXPECT_NEAR(sum / N, 0.0, 0.05);
}

TEST(GaussianNoiseModel, StdDevMatchesSigma) {
  const double sigma = 0.5;
  GaussianNoiseModel m(sigma);
  double sum = 0.0, sum_sq = 0.0;
  const int N = 10000;
  for (int i = 0; i < N; ++i) {
    double s = m.sample();
    sum += s;
    sum_sq += s * s;
  }
  double mean = sum / N;
  double estimated_std = std::sqrt(sum_sq / N - mean * mean);
  EXPECT_NEAR(estimated_std, sigma, 0.05);
}

TEST(GaussianNoiseModel, ProducesPositiveAndNegativeSamples) {
  GaussianNoiseModel m(1.0);
  bool has_positive = false, has_negative = false;
  for (int i = 0; i < 200; ++i) {
    double s = m.sample();
    if (s > 0) {has_positive = true;}
    if (s < 0) {has_negative = true;}
  }
  EXPECT_TRUE(has_positive);
  EXPECT_TRUE(has_negative);
}

// =============================================================================
// NLOSBiasModel
// =============================================================================

TEST(NLOSBiasModel, ZeroProbabilityAlwaysZero) {
  NLOSBiasModel m(1.0, 0.0);  // p_nlos=0 → never fires
  for (int i = 0; i < 200; ++i) {
    EXPECT_DOUBLE_EQ(m.sample(), 0.0);
  }
}

TEST(NLOSBiasModel, AlwaysNonNegative) {
  NLOSBiasModel m(2.0, 0.5);
  for (int i = 0; i < 1000; ++i) {
    EXPECT_GE(m.sample(), 0.0);
  }
}

TEST(NLOSBiasModel, FullProbabilityHasPositiveMean) {
  NLOSBiasModel m(1.0, 1.0);  // Always NLOS; mean of Exp(lambda=1) = 1.0
  double sum = 0.0;
  const int N = 1000;
  for (int i = 0; i < N; ++i) {
    sum += m.sample();
  }
  EXPECT_GT(sum / N, 0.5);
}

TEST(NLOSBiasModel, HigherLambdaGivesSmallBias) {
  // Exp(lambda) has mean = 1/lambda; higher lambda → smaller mean
  NLOSBiasModel low(1.0, 1.0);   // mean ≈ 1.0
  NLOSBiasModel high(10.0, 1.0);  // mean ≈ 0.1
  double sum_low = 0.0, sum_high = 0.0;
  const int N = 2000;
  for (int i = 0; i < N; ++i) {
    sum_low += low.sample();
    sum_high += high.sample();
  }
  EXPECT_GT(sum_low / N, sum_high / N);
}

// =============================================================================
// MultipathModel
// =============================================================================

TEST(MultipathModel, ZeroParamsAlwaysZero) {
  MultipathModel m(0.0, 0.0);
  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(m.sample(), 0.0);
  }
}

TEST(MultipathModel, BoundedForStableAlpha) {
  MultipathModel m(0.5, 0.1);  // Stable AR(1): |alpha| < 1
  for (int i = 0; i < 1000; ++i) {
    EXPECT_LT(std::abs(m.sample()), 5.0);
  }
}

TEST(MultipathModel, ProducesVariance) {
  MultipathModel m(0.5, 0.2);
  double sum = 0.0, sum_sq = 0.0;
  const int N = 1000;
  for (int i = 0; i < N; ++i) {
    double s = m.sample();
    sum += s;
    sum_sq += s * s;
  }
  double variance = sum_sq / N - (sum / N) * (sum / N);
  EXPECT_GT(variance, 0.0);
}

// =============================================================================
// ClockDriftModel
// =============================================================================

TEST(ClockDriftModel, ZeroSigmaStaysZero) {
  ClockDriftModel m(0.0);
  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(m.update(), 0.0);
    EXPECT_DOUBLE_EQ(m.getDrift(), 0.0);
  }
}

TEST(ClockDriftModel, DriftChangesOverTime) {
  ClockDriftModel m(0.01);
  int changes = 0;
  double prev = m.getDrift();
  for (int i = 0; i < 100; ++i) {
    m.update();
    if (m.getDrift() != prev) {changes++;}
    prev = m.getDrift();
  }
  EXPECT_GT(changes, 0);
}

TEST(ClockDriftModel, GetDriftMatchesLastUpdate) {
  ClockDriftModel m(0.05);
  double last = m.update();
  EXPECT_DOUBLE_EQ(m.getDrift(), last);
}

// =============================================================================
// ChannelModel
// =============================================================================

TEST(ChannelModel, MeasuredEqualsComponentSum) {
  ChannelParams p;
  p.gaussian_sigma = 0.1;
  p.nlos_lambda = 1.0;
  p.nlos_prob = 0.3;
  p.multipath_alpha = 0.3;
  p.multipath_sigma = 0.05;
  p.clock_drift_sigma = 0.001;

  ChannelModel m(p);
  Eigen::Vector3d anchor(0, 0, 0), tag(3, 4, 0);  // true distance = 5.0

  for (int i = 0; i < 200; ++i) {
    UWBMeasurement meas = m.computeDetailedMeasurement(anchor, tag);
    double expected = meas.d_true + meas.gaussian_noise +
      meas.nlos_bias + meas.multipath_error + meas.clock_drift;
    EXPECT_NEAR(meas.measured, expected, 1e-9);
    EXPECT_NEAR(meas.d_true, 5.0, 1e-9);
  }
}

TEST(ChannelModel, NLOSBiasAlwaysNonNegative) {
  ChannelParams p;
  p.gaussian_sigma = 0.0;
  p.nlos_lambda = 2.0;
  p.nlos_prob = 1.0;          // Always NLOS
  p.multipath_alpha = 0.0;
  p.multipath_sigma = 0.0;
  p.clock_drift_sigma = 0.0;

  ChannelModel m(p);
  Eigen::Vector3d anchor(0, 0, 0), tag(5, 0, 0);

  for (int i = 0; i < 300; ++i) {
    UWBMeasurement meas = m.computeDetailedMeasurement(anchor, tag);
    EXPECT_GE(meas.nlos_bias, 0.0);
    EXPECT_GE(meas.measured, meas.d_true);  // NLOS only adds positive bias
  }
}

TEST(ChannelModel, ZeroNoiseModelIsExact) {
  ChannelParams p;
  p.gaussian_sigma = 0.0;
  p.nlos_lambda = 1.0;
  p.nlos_prob = 0.0;          // Never NLOS
  p.multipath_alpha = 0.0;
  p.multipath_sigma = 0.0;
  p.clock_drift_sigma = 0.0;

  ChannelModel m(p);
  Eigen::Vector3d anchor(0, 0, 0), tag(3, 4, 0);  // true = 5.0

  for (int i = 0; i < 50; ++i) {
    EXPECT_NEAR(m.computeMeasuredRange(anchor, tag), 5.0, 1e-9);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

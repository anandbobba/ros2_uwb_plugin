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
// Unit tests for the Gauss-Newton trilateration solver.
// Tests the core algorithm in isolation (no ROS runtime needed).
// The solver logic mirrors TrilaterationNode::solve_trilateration exactly.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

// ---------------------------------------------------------------------------
// Standalone Gauss-Newton solver — mirrors TrilaterationNode::solve_trilateration
// ---------------------------------------------------------------------------
struct TestAnchor
{
  Eigen::Vector3d position;
  double range;
};

Eigen::Vector3d gauss_newton(
  const std::vector<TestAnchor> & anchors,
  Eigen::Vector3d x0 = Eigen::Vector3d::Zero(),
  int max_iter = 50,
  double conv_thresh = 1e-7)
{
  const size_t n = anchors.size();
  Eigen::Vector3d x = x0;

  for (int iter = 0; iter < max_iter; ++iter) {
    Eigen::MatrixXd J(n, 3);
    Eigen::VectorXd r(n);

    for (size_t j = 0; j < n; ++j) {
      Eigen::Vector3d diff = x - anchors[j].position;
      double d = diff.norm();
      if (d < 1e-6) {d = 1e-6;}
      J.row(j) = diff.transpose() / d;
      r(j) = anchors[j].range - d;
    }

    Eigen::Matrix3d JtJ = J.transpose() * J;
    Eigen::Vector3d dx = JtJ.ldlt().solve(J.transpose() * r);
    x += dx;

    if (dx.norm() < conv_thresh) {break;}
  }
  return x;
}

// Compute exact ranges from a known position to a set of anchor positions
std::vector<TestAnchor> make_anchors(
  const std::vector<Eigen::Vector3d> & positions,
  const Eigen::Vector3d & tag_pos,
  double noise = 0.0)
{
  std::vector<TestAnchor> anchors;
  for (size_t i = 0; i < positions.size(); ++i) {
    double sign = (i % 2 == 0) ? 1.0 : -1.0;
    anchors.push_back(
      {positions[i],
        (tag_pos - positions[i]).norm() + sign * noise});
  }
  return anchors;
}

// =============================================================================
// Convergence with perfect (noiseless) ranges
// =============================================================================

TEST(Trilateration, OriginPerfectRanges) {
  Eigen::Vector3d truth(0, 0, 0);
  auto anchors = make_anchors(
  {
    {5, 0, 0}, {-5, 0, 0}, {0, 5, 0}, {0, -5, 0}}, truth);

  Eigen::Vector3d est = gauss_newton(anchors);
  EXPECT_NEAR(est.x(), truth.x(), 1e-4);
  EXPECT_NEAR(est.y(), truth.y(), 1e-4);
}

TEST(Trilateration, ArbitraryPositionPerfectRanges) {
  Eigen::Vector3d truth(2, 3, 0);
  auto anchors = make_anchors(
  {
    {0, 0, 0}, {10, 0, 0}, {0, 10, 0}, {5, 5, 0}}, truth);

  Eigen::Vector3d est = gauss_newton(anchors);
  EXPECT_NEAR(est.x(), truth.x(), 1e-4);
  EXPECT_NEAR(est.y(), truth.y(), 1e-4);
}

TEST(Trilateration, NegativeCoordinates) {
  Eigen::Vector3d truth(-3, -2, 0);
  auto anchors = make_anchors(
  {
    {0, 0, 0}, {-10, 0, 0}, {0, -10, 0}, {-5, -5, 0}}, truth);

  Eigen::Vector3d est = gauss_newton(anchors);
  EXPECT_NEAR(est.x(), truth.x(), 1e-4);
  EXPECT_NEAR(est.y(), truth.y(), 1e-4);
}

// =============================================================================
// Minimum anchor count (3)
// =============================================================================

TEST(Trilateration, ThreeAnchorsConverges) {
  Eigen::Vector3d truth(1.5, 1.5, 0);
  auto anchors = make_anchors(
  {
    {0, 0, 0}, {5, 0, 0}, {0, 5, 0}}, truth);

  Eigen::Vector3d est = gauss_newton(anchors);
  EXPECT_NEAR(est.x(), truth.x(), 1e-3);
  EXPECT_NEAR(est.y(), truth.y(), 1e-3);
}

// =============================================================================
// Robustness with noisy ranges
// =============================================================================

TEST(Trilateration, SmallNoise5cm) {
  Eigen::Vector3d truth(1, 2, 0);
  auto anchors = make_anchors(
  {
    {0, 0, 0}, {8, 0, 0}, {0, 8, 0}, {8, 8, 0}}, truth, 0.05);

  Eigen::Vector3d est = gauss_newton(anchors);
  EXPECT_LT((est - truth).norm(), 0.10);  // Within 10 cm
}

TEST(Trilateration, LargerNoise20cm) {
  Eigen::Vector3d truth(3, 3, 0);
  auto anchors = make_anchors(
  {
    {0, 0, 0}, {10, 0, 0}, {0, 10, 0}, {10, 10, 0}}, truth, 0.20);

  Eigen::Vector3d est = gauss_newton(anchors);
  EXPECT_LT((est - truth).norm(), 0.40);  // Within 40 cm (2× noise)
}

// =============================================================================
// Initial guess independence
// =============================================================================

TEST(Trilateration, ConvergesFromDistantGuess) {
  Eigen::Vector3d truth(3, 4, 0);
  auto anchors = make_anchors(
  {
    {0, 0, 0}, {10, 0, 0}, {0, 10, 0}, {5, 5, 0}}, truth);

  Eigen::Vector3d est = gauss_newton(anchors, Eigen::Vector3d(8, 8, 0));
  EXPECT_NEAR(est.x(), truth.x(), 1e-4);
  EXPECT_NEAR(est.y(), truth.y(), 1e-4);
}

TEST(Trilateration, ConvergesFromZeroInit) {
  Eigen::Vector3d truth(7, 6, 0);
  auto anchors = make_anchors(
  {
    {0, 0, 0}, {15, 0, 0}, {0, 15, 0}, {15, 15, 0}}, truth);

  Eigen::Vector3d est = gauss_newton(anchors, Eigen::Vector3d::Zero());
  EXPECT_NEAR(est.x(), truth.x(), 1e-4);
  EXPECT_NEAR(est.y(), truth.y(), 1e-4);
}

// =============================================================================
// Residual property: perfect ranges → near-zero residuals after solve
// =============================================================================

TEST(Trilateration, PerfectRangesHaveNearZeroResiduals) {
  Eigen::Vector3d truth(4, 1, 0);
  auto anchors = make_anchors(
  {
    {0, 0, 0}, {10, 0, 0}, {0, 10, 0}, {10, 10, 0}}, truth);

  Eigen::Vector3d est = gauss_newton(anchors);

  for (const auto & a : anchors) {
    double residual = std::abs(a.range - (est - a.position).norm());
    EXPECT_LT(residual, 1e-4);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

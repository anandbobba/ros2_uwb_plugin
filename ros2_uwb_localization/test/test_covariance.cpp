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
// Unit tests for the covariance propagation math in TrilaterationNode.
// The relationship tested is:  Cov = sigma^2 * (J^T J)^{-1}
// All tests run in isolation with no ROS runtime.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

// ---------------------------------------------------------------------------
// Helpers — mirror exactly what TrilaterationNode does
// ---------------------------------------------------------------------------
struct TestAnchor
{
  Eigen::Vector3d position;
  double range;
};

/// Build Jacobian for a given estimate position and anchor set.
Eigen::MatrixXd build_jacobian(
  const Eigen::Vector3d & x,
  const std::vector<TestAnchor> & anchors)
{
  const size_t n = anchors.size();
  Eigen::MatrixXd J(n, 3);
  for (size_t j = 0; j < n; ++j) {
    Eigen::Vector3d diff = x - anchors[j].position;
    double d = diff.norm();
    if (d < 1e-6) {d = 1e-6;}
    J.row(j) = diff.transpose() / d;
  }
  return J;
}

/// Compute covariance as sigma^2 * (J^T J)^{-1}.
Eigen::Matrix3d compute_covariance(
  const Eigen::MatrixXd & J,
  double sigma)
{
  Eigen::Matrix3d JtJ = J.transpose() * J;
  return sigma * sigma * JtJ.inverse();
}

/// Near-singular check (mirrors TrilaterationNode logic).
bool is_near_singular(const Eigen::Matrix3d & H, double tol = 1e-10)
{
  return std::abs(H.determinant()) < tol;
}

// =============================================================================
// Basic covariance properties
// =============================================================================

TEST(Covariance, MatrixIsSymmetric) {
  // 6 anchors at corners of an octahedron, tag at origin
  std::vector<TestAnchor> anchors = {
    {{5, 0, 0}, 5.0},
    {{-5, 0, 0}, 5.0},
    {{0, 5, 0}, 5.0},
    {{0, -5, 0}, 5.0},
    {{0, 0, 5}, 5.0},
    {{0, 0, -5}, 5.0},
  };
  Eigen::Vector3d x(0, 0, 0);
  auto J = build_jacobian(x, anchors);
  auto cov = compute_covariance(J, 0.1);

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(cov(i, j), cov(j, i), 1e-10);
    }
  }
}

TEST(Covariance, MatrixIsPositiveDefinite) {
  std::vector<TestAnchor> anchors = {
    {{5, 0, 0}, 5.0},
    {{-5, 0, 0}, 5.0},
    {{0, 5, 0}, 5.0},
    {{0, -5, 0}, 5.0},
    {{0, 0, 5}, 5.0},
    {{0, 0, -5}, 5.0},
  };
  Eigen::Vector3d x(0, 0, 0);
  auto J = build_jacobian(x, anchors);
  auto cov = compute_covariance(J, 0.1);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
  for (int i = 0; i < 3; ++i) {
    EXPECT_GT(solver.eigenvalues()(i), 0.0);
  }
}

TEST(Covariance, LargerSigmaGivesLargerCovariance) {
  // Same anchors, different sigma — all eigenvalues should scale as sigma^2
  std::vector<TestAnchor> anchors = {
    {{5, 0, 0}, 5.0},
    {{-5, 0, 0}, 5.0},
    {{0, 5, 0}, 5.0},
    {{0, -5, 0}, 5.0},
    {{0, 0, 5}, 5.0},
    {{0, 0, -5}, 5.0},
  };
  Eigen::Vector3d x(1, 1, 1);
  auto J = build_jacobian(x, anchors);
  auto cov_small = compute_covariance(J, 0.05);
  auto cov_large = compute_covariance(J, 0.30);

  // Trace (sum of eigenvalues) should be larger for larger sigma
  EXPECT_GT(cov_large.trace(), cov_small.trace());

  // Scaling ratio should be (0.30/0.05)^2 = 36
  EXPECT_NEAR(cov_large.trace() / cov_small.trace(), 36.0, 1e-6);
}

// =============================================================================
// Hand-calculated reference case
// =============================================================================

TEST(Covariance, KnownFourAnchorSymmetricCase) {
  // Anchors at (±d, 0, 0), (0, ±d, 0), and (0, 0, ±d) with tag at origin.
  // By symmetry, J^T J for the 3D case converges to (n/3)*I_3 for large n.
  // For d=5 and tag at origin, each row of J is ±[1,0,0], ±[0,1,0] or ±[0,0,1].
  // J^T J = diag(2, 2, 2) for the 3 planes,
  // so the x,y,z covariance = sigma^2/2.
  std::vector<TestAnchor> anchors = {
    {{5, 0, 0}, 5.0},
    {{-5, 0, 0}, 5.0},
    {{0, 5, 0}, 5.0},
    {{0, -5, 0}, 5.0},
    {{0, 0, 5}, 5.0},
    {{0, 0, -5}, 5.0},
  };
  const double sigma = 0.1;
  Eigen::Vector3d x(0, 0, 0);
  auto J = build_jacobian(x, anchors);
  auto cov = compute_covariance(J, sigma);

  // x,y covariance must be sigma^2/2
  EXPECT_NEAR(cov(0, 0), sigma * sigma / 2.0, 1e-6);
  EXPECT_NEAR(cov(1, 1), sigma * sigma / 2.0, 1e-6);
  // Cross terms must be near zero by symmetry
  EXPECT_NEAR(cov(0, 1), 0.0, 1e-6);
  EXPECT_NEAR(cov(1, 0), 0.0, 1e-6);
}

// =============================================================================
// Near-singular Hessian guard
// =============================================================================

TEST(Covariance, IdentityHessianIsNotSingular) {
  Eigen::Matrix3d H = Eigen::Matrix3d::Identity();
  EXPECT_FALSE(is_near_singular(H));
}

TEST(Covariance, ZeroHessianIsSingular) {
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  EXPECT_TRUE(is_near_singular(H));
}

TEST(Covariance, RankDeficientHessianIsSingular) {
  // Two anchors on a line — J^T J is rank 1 in the degenerate direction
  // Use a nearly-zero determinant directly
  Eigen::Matrix3d H;
  H << 1, 0, 0,
    0, 1, 0,
    0, 0, 1e-12;  // near-zero det
  EXPECT_TRUE(is_near_singular(H));
}

TEST(Covariance, WellConditionedHessianIsNotSingular) {
  // 6 anchors in 3D give full-rank J^T J
  std::vector<TestAnchor> anchors = {
    {{5, 0, 0}, 5.0},
    {{-5, 0, 0}, 5.0},
    {{0, 5, 0}, 5.0},
    {{0, -5, 0}, 5.0},
    {{0, 0, 5}, 5.0},
    {{0, 0, -5}, 5.0},
  };
  Eigen::Vector3d x(1, 1, 1);
  auto J = build_jacobian(x, anchors);
  Eigen::Matrix3d H = J.transpose() * J;
  EXPECT_FALSE(is_near_singular(H));
}

// =============================================================================
// Outlier rejection — counts
// =============================================================================

TEST(OutlierRejection, AllInliersWithPerfectRanges) {
  // If ranges are perfect, no anchor should be rejected
  std::vector<TestAnchor> anchors = {
    {{5, 0, 0}, 5.0},
    {{-5, 0, 0}, 5.0},
    {{0, 5, 0}, 5.0},
    {{0, -5, 0}, 5.0},
  };
  Eigen::Vector3d x(0, 0, 0);
  const double outlier_threshold = 1.0;
  int outlier_count = 0;
  for (const auto & a : anchors) {
    double residual = std::abs(a.range - (x - a.position).norm());
    if (residual > outlier_threshold) {outlier_count++;}
  }
  EXPECT_EQ(outlier_count, 0);
}

TEST(OutlierRejection, LargeRangeErrorIsRejected) {
  // One anchor has a grossly wrong range
  std::vector<TestAnchor> anchors = {
    {{5, 0, 0}, 5.0},
    {{-5, 0, 0}, 5.0},
    {{0, 5, 0}, 5.0},
    {{0, -5, 0}, 50.0},   // Outlier: true range is 5m but reported as 50m
  };
  Eigen::Vector3d x(0, 0, 0);
  const double outlier_threshold = 2.0;
  int outlier_count = 0;
  for (const auto & a : anchors) {
    double residual = std::abs(a.range - (x - a.position).norm());
    if (residual > outlier_threshold) {outlier_count++;}
  }
  EXPECT_EQ(outlier_count, 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

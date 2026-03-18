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
// Unit tests for AnchorManager geometry and configuration logic.
// Tests the core data structures and transform math in isolation —
// no ROS runtime or TF2 broadcaster is required.

#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Minimal replica of AnchorConfig (mirrors anchor_manager_node.hpp)
// ---------------------------------------------------------------------------
struct AnchorConfig
{
  std::string id;
  Eigen::Vector3d position;
};

// ---------------------------------------------------------------------------
// Minimal replica of a transform record (mirrors geometry_msgs::TransformStamped)
// ---------------------------------------------------------------------------
struct TransformRecord
{
  std::string frame_id;
  std::string child_frame_id;
  Eigen::Vector3d translation;
};

// Simulate what AnchorManagerNode::broadcast_transforms() builds
std::vector<TransformRecord> build_transforms(
  const std::vector<AnchorConfig> & anchors,
  const std::string & world_frame)
{
  std::vector<TransformRecord> out;
  for (const auto & a : anchors) {
    TransformRecord t;
    t.frame_id = world_frame;
    t.child_frame_id = a.id;
    t.translation = a.position;
    out.push_back(t);
  }
  return out;
}

// =============================================================================
// AnchorConfig construction
// =============================================================================

TEST(AnchorConfig, DefaultPosition) {
  AnchorConfig a;
  a.id = "uwb_anchor_0";
  a.position = Eigen::Vector3d(5.0, 5.0, 2.0);
  EXPECT_EQ(a.id, "uwb_anchor_0");
  EXPECT_DOUBLE_EQ(a.position.x(), 5.0);
  EXPECT_DOUBLE_EQ(a.position.y(), 5.0);
  EXPECT_DOUBLE_EQ(a.position.z(), 2.0);
}

TEST(AnchorConfig, NegativeCoordinates) {
  AnchorConfig a;
  a.id = "uwb_anchor_1";
  a.position = Eigen::Vector3d(-5.0, -3.0, 2.5);
  EXPECT_DOUBLE_EQ(a.position.x(), -5.0);
  EXPECT_DOUBLE_EQ(a.position.y(), -3.0);
  EXPECT_DOUBLE_EQ(a.position.z(), 2.5);
}

TEST(AnchorConfig, ZeroPosition) {
  AnchorConfig a;
  a.id = "origin_anchor";
  a.position = Eigen::Vector3d::Zero();
  EXPECT_DOUBLE_EQ(a.position.norm(), 0.0);
}

// =============================================================================
// Transform generation — frame IDs
// =============================================================================

TEST(BroadcastTransforms, EmptyAnchorsProducesNoTransforms) {
  std::vector<AnchorConfig> anchors;
  auto transforms = build_transforms(anchors, "map");
  EXPECT_TRUE(transforms.empty());
}

TEST(BroadcastTransforms, SingleAnchorFrameIds) {
  AnchorConfig a;
  a.id = "uwb_anchor_0";
  a.position = Eigen::Vector3d(1.0, 2.0, 3.0);

  auto transforms = build_transforms({a}, "map");
  ASSERT_EQ(transforms.size(), 1u);
  EXPECT_EQ(transforms[0].frame_id, "map");
  EXPECT_EQ(transforms[0].child_frame_id, "uwb_anchor_0");
}

TEST(BroadcastTransforms, TranslationMatchesPosition) {
  AnchorConfig a;
  a.id = "test_anchor";
  a.position = Eigen::Vector3d(3.5, -1.2, 2.0);

  auto transforms = build_transforms({a}, "world");
  ASSERT_EQ(transforms.size(), 1u);
  EXPECT_NEAR(transforms[0].translation.x(), 3.5, 1e-9);
  EXPECT_NEAR(transforms[0].translation.y(), -1.2, 1e-9);
  EXPECT_NEAR(transforms[0].translation.z(), 2.0, 1e-9);
}

TEST(BroadcastTransforms, FourAnchorsProduceFourTransforms) {
  std::vector<AnchorConfig> anchors;
  for (int i = 0; i < 4; ++i) {
    AnchorConfig a;
    a.id = "uwb_anchor_" + std::to_string(i);
    a.position = Eigen::Vector3d(i * 2.0, 0.0, 2.0);
    anchors.push_back(a);
  }

  auto transforms = build_transforms(anchors, "map");
  ASSERT_EQ(transforms.size(), 4u);
  for (size_t i = 0; i < transforms.size(); ++i) {
    EXPECT_EQ(transforms[i].frame_id, "map");
    EXPECT_EQ(transforms[i].child_frame_id, anchors[i].id);
    EXPECT_NEAR(transforms[i].translation.x(), i * 2.0, 1e-9);
  }
}

TEST(BroadcastTransforms, CustomWorldFrame) {
  AnchorConfig a;
  a.id = "test";
  a.position = Eigen::Vector3d::Zero();
  auto transforms = build_transforms({a}, "odom");
  ASSERT_EQ(transforms.size(), 1u);
  EXPECT_EQ(transforms[0].frame_id, "odom");
}

// =============================================================================
// Anchor centroid (used for initial estimate seeding)
// =============================================================================

TEST(AnchorCentroid, FourSymmetricAnchors) {
  // Anchors at (±5, ±5, 2) — centroid should be (0, 0, 2)
  std::vector<AnchorConfig> anchors = {
    {"a0", Eigen::Vector3d(5, 5, 2)},
    {"a1", Eigen::Vector3d(-5, 5, 2)},
    {"a2", Eigen::Vector3d(5, -5, 2)},
    {"a3", Eigen::Vector3d(-5, -5, 2)},
  };
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  for (const auto & a : anchors) {
    centroid += a.position;
  }
  centroid /= static_cast<double>(anchors.size());
  EXPECT_NEAR(centroid.x(), 0.0, 1e-9);
  EXPECT_NEAR(centroid.y(), 0.0, 1e-9);
  EXPECT_NEAR(centroid.z(), 2.0, 1e-9);
}

TEST(AnchorCentroid, SingleAnchorCentroidIsItself) {
  AnchorConfig a;
  a.id = "solo";
  a.position = Eigen::Vector3d(3.0, 7.0, 1.5);
  Eigen::Vector3d centroid = a.position / 1.0;
  EXPECT_NEAR(centroid.x(), 3.0, 1e-9);
  EXPECT_NEAR(centroid.y(), 7.0, 1e-9);
  EXPECT_NEAR(centroid.z(), 1.5, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

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

#include <gtest/gtest.h>
#include <cmath>

// Simple math test for the yaw calculation
TEST(YawEstimatorTest, ComputeYaw)
{
  double front_x = 1.0;
  double front_y = 0.0;
  double rear_x = 0.0;
  double rear_y = 0.0;

  double dx = front_x - rear_x;
  double dy = front_y - rear_y;
  double distance = std::sqrt(dx * dx + dy * dy);

  EXPECT_DOUBLE_EQ(distance, 1.0);
  
  double raw_yaw = std::atan2(dy, dx);
  EXPECT_DOUBLE_EQ(raw_yaw, 0.0);

  // 90 degrees
  front_x = 0.0;
  front_y = 1.0;
  dx = front_x - rear_x;
  dy = front_y - rear_y;
  raw_yaw = std::atan2(dy, dx);
  EXPECT_DOUBLE_EQ(raw_yaw, M_PI / 2.0);
}

// EMA test
TEST(YawEstimatorTest, EMASmoothing)
{
  double smoothing_alpha = 0.8;
  double current_yaw = 0.0;
  double raw_yaw = M_PI / 2.0;

  double current_sin = std::sin(current_yaw);
  double current_cos = std::cos(current_yaw);
  double raw_sin = std::sin(raw_yaw);
  double raw_cos = std::cos(raw_yaw);
  
  double new_sin = smoothing_alpha * raw_sin + (1.0 - smoothing_alpha) * current_sin;
  double new_cos = smoothing_alpha * raw_cos + (1.0 - smoothing_alpha) * current_cos;
  
  double smoothed_yaw = std::atan2(new_sin, new_cos);
  
  // atan2(0.8, 0.2) = 1.3258 rad
  EXPECT_NEAR(smoothed_yaw, 1.32581766, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

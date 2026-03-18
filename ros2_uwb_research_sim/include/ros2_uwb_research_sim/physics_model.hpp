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

#ifndef ROS2_UWB_RESEARCH_SIM__PHYSICS_MODEL_HPP_
#define ROS2_UWB_RESEARCH_SIM__PHYSICS_MODEL_HPP_

#include <Eigen/Dense>

namespace ros2_uwb_research_sim
{

class PhysicsModel
{
public:
  PhysicsModel() = default;

  /**
   * @brief Computes true Euclidean distance between two points.
   */
  double computeDistance(const Eigen::Vector3d & pos1, const Eigen::Vector3d & pos2) const;
};

}  // namespace ros2_uwb_research_sim

#endif  // ROS2_UWB_RESEARCH_SIM__PHYSICS_MODEL_HPP_

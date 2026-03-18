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

#ifndef ROS2_UWB_RESEARCH_SIM__CHANNEL_MODEL_HPP_
#define ROS2_UWB_RESEARCH_SIM__CHANNEL_MODEL_HPP_

#include <memory>
#include "physics_model.hpp"
#include "gaussian_noise_model.hpp"
#include "nlos_bias_model.hpp"
#include "multipath_model.hpp"
#include "clock_drift_model.hpp"

namespace ros2_uwb_research_sim
{

struct ChannelParams
{
  double gaussian_sigma;
  double nlos_lambda;
  double nlos_prob;
  double multipath_alpha;
  double multipath_sigma;
  double clock_drift_sigma;
};

struct UWBMeasurement
{
  double d_true;
  double measured;
  double gaussian_noise;
  double nlos_bias;
  double multipath_error;
  double clock_drift;
};

class ChannelModel
{
public:
  explicit ChannelModel(const ChannelParams & params);

  /**
   * @brief Factory method to get presets for standard environments.
   * @param profile One of: "ideal", "indoor", "warehouse", "research"
   */
  static ChannelParams getParamsByProfile(const std::string & profile);

  double computeMeasuredRange(const Eigen::Vector3d & anchor_pos, const Eigen::Vector3d & tag_pos);
  UWBMeasurement computeDetailedMeasurement(
    const Eigen::Vector3d & anchor_pos,
    const Eigen::Vector3d & tag_pos);
  
  void setParams(const ChannelParams & params);
  const ChannelParams & getParams() const {return params_;}

private:
  ChannelParams params_;
  PhysicsModel physics_;
  std::unique_ptr<GaussianNoiseModel> gaussian_noise_;
  std::unique_ptr<NLOSBiasModel> nlos_bias_;
  std::unique_ptr<MultipathModel> multipath_;
  std::unique_ptr<ClockDriftModel> clock_drift_;
};

}  // namespace ros2_uwb_research_sim

#endif  // ROS2_UWB_RESEARCH_SIM__CHANNEL_MODEL_HPP_

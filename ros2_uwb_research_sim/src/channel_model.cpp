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

#include "ros2_uwb_research_sim/channel_model.hpp"

namespace ros2_uwb_research_sim
{

ChannelModel::ChannelModel(const ChannelParams & params)
  : params_(params)
{
  gaussian_noise_ = std::make_unique<GaussianNoiseModel>(params.gaussian_sigma);
  nlos_bias_ = std::make_unique<NLOSBiasModel>(params.nlos_lambda, params.nlos_prob);
  multipath_ = std::make_unique<MultipathModel>(params.multipath_alpha, params.multipath_sigma);
  clock_drift_ = std::make_unique<ClockDriftModel>(params.clock_drift_sigma);
}

ChannelParams ChannelModel::getParamsByProfile(const std::string & profile)
{
  ChannelParams p;
  if (profile == "ideal") {
    p.gaussian_sigma = 0.01;
    p.nlos_lambda = 0.0;
    p.nlos_prob = 0.0;
    p.multipath_alpha = 0.0;
    p.multipath_sigma = 0.0;
    p.clock_drift_sigma = 0.0;
  } else if (profile == "indoor") {
    p.gaussian_sigma = 0.02;     // 2cm std
    p.nlos_lambda = 0.5;        // Mild bias
    p.nlos_prob = 0.05;         // 5% chance NLOS
    p.multipath_alpha = 0.95;   // Slow multipath variation
    p.multipath_sigma = 0.01;
    p.clock_drift_sigma = 0.00001;
  } else if (profile == "warehouse") {
    p.gaussian_sigma = 0.05;     // 5cm std
    p.nlos_lambda = 2.0;        // Heavy bias
    p.nlos_prob = 0.20;         // 20% chance NLOS
    p.multipath_alpha = 0.80;   // Faster multipath
    p.multipath_sigma = 0.05;
    p.clock_drift_sigma = 0.0001;
  } else {  // "research" (default legacy settings)
    p.gaussian_sigma = 0.02;
    p.nlos_lambda = 1.5;
    p.nlos_prob = 0.1;
    p.multipath_alpha = 0.8;
    p.multipath_sigma = 0.01;
    p.clock_drift_sigma = 0.0001;
  }
  return p;
}

double ChannelModel::computeMeasuredRange(
  const Eigen::Vector3d & anchor_pos,
  const Eigen::Vector3d & tag_pos)
{
  double d_true = physics_.computeDistance(anchor_pos, tag_pos);

  double epsilon_gaussian = gaussian_noise_->sample();
  double epsilon_nlos = nlos_bias_->sample();
  double epsilon_multipath = multipath_->sample();
  double epsilon_drift = clock_drift_->update();

  return d_true + epsilon_gaussian + epsilon_nlos + epsilon_multipath + epsilon_drift;
}

UWBMeasurement ChannelModel::computeDetailedMeasurement(
  const Eigen::Vector3d & anchor_pos,
  const Eigen::Vector3d & tag_pos)
{
  UWBMeasurement m;
  m.d_true = physics_.computeDistance(anchor_pos, tag_pos);
  m.gaussian_noise = gaussian_noise_->sample();
  m.nlos_bias = nlos_bias_->sample();
  m.multipath_error = multipath_->sample();
  m.clock_drift = clock_drift_->update();
  m.measured = m.d_true + m.gaussian_noise + m.nlos_bias + m.multipath_error + m.clock_drift;
  return m;
}

void ChannelModel::setParams(const ChannelParams & params)
{
  params_ = params;
  gaussian_noise_ = std::make_unique<GaussianNoiseModel>(params.gaussian_sigma);
  nlos_bias_ = std::make_unique<NLOSBiasModel>(params.nlos_lambda, params.nlos_prob);
  multipath_ = std::make_unique<MultipathModel>(params.multipath_alpha, params.multipath_sigma);
  clock_drift_ = std::make_unique<ClockDriftModel>(params.clock_drift_sigma);
}

}  // namespace ros2_uwb_research_sim

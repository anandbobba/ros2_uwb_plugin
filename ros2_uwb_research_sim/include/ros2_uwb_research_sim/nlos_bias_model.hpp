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

#ifndef ROS2_UWB_RESEARCH_SIM__NLOS_BIAS_MODEL_HPP_
#define ROS2_UWB_RESEARCH_SIM__NLOS_BIAS_MODEL_HPP_

#include <random>

namespace ros2_uwb_research_sim
{

class NLOSBiasModel
{
public:
  explicit NLOSBiasModel(double lambda, double p_nlos);

  double sample();

private:
  std::mt19937 gen_;
  std::exponential_distribution<double> exp_dist_;
  std::bernoulli_distribution bern_dist_;
};

}  // namespace ros2_uwb_research_sim

#endif  // ROS2_UWB_RESEARCH_SIM__NLOS_BIAS_MODEL_HPP_

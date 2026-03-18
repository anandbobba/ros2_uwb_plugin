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

#include "ros2_uwb_research_sim/gaussian_noise_model.hpp"

namespace ros2_uwb_research_sim
{

GaussianNoiseModel::GaussianNoiseModel(double sigma)
: gen_(std::random_device{}()), dist_(0.0, sigma) {}

double GaussianNoiseModel::sample()
{
  return dist_(gen_);
}

}  // namespace ros2_uwb_research_sim

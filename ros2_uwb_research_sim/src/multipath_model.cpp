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

#include "ros2_uwb_research_sim/multipath_model.hpp"

namespace ros2_uwb_research_sim
{

MultipathModel::MultipathModel(double alpha, double sigma_mp)
: alpha_(alpha), last_error_(0.0), gen_(std::random_device{}()), dist_(0.0, sigma_mp) {}

double MultipathModel::sample()
{
  double w_t = dist_(gen_);
  last_error_ = alpha_ * last_error_ + w_t;
  return last_error_;
}

}  // namespace ros2_uwb_research_sim

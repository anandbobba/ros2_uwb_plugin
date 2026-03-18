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

#include "ros2_uwb_research_sim/clock_drift_model.hpp"

namespace ros2_uwb_research_sim
{

ClockDriftModel::ClockDriftModel(double sigma_drift)
: current_drift_(0.0), gen_(std::random_device{}()), dist_(0.0, sigma_drift) {}

double ClockDriftModel::update()
{
  current_drift_ += dist_(gen_);
  return current_drift_;
}

double ClockDriftModel::getDrift() const
{
  return current_drift_;
}

}  // namespace ros2_uwb_research_sim

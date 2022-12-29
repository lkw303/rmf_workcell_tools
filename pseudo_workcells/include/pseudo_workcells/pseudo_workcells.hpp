// Copyright 2022 ROS Industrial Consortium Asia Pacific
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

#include <memory>
#include <string>
#include <vector>
#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "workcell_triggers/ingestor_trigger.hpp"
#include "workcell_triggers/dispenser_trigger.hpp"

namespace pseudo_workcells{

class PseudoWorkcells
{
private:
  std::vector<std::string> ingestor_names;
  std::vector<std::string> dispenser_names;
  std::vector<std::shared_ptr<workcell_triggers::IngestorTrigger>> ingestors;
  std::vector<std::shared_ptr<workcell_triggers::DispenserTrigger>> dispensers;
  rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::Node::SharedPtr node;

public:
  PseudoWorkcells(const rclcpp::Node::SharedPtr _node);
  void spin_all();
  void init_workcells();

};

} // namespace pseudo_workcells
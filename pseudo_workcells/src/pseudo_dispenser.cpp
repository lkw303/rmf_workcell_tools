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

#include "pseudo_workcells/pseudo_dispenser.hpp"


namespace pseudo_workcells{
  PseudoDispenser::PseudoDispenser(const rclcpp::Node::SharedPtr _node)
  {
    node = std::move(_node);
    node->declare_parameter("workcell_name", std::string{""});
    dispenser_name = node->get_parameter("workcell_name").as_string();
    RCLCPP_INFO(node->get_logger(),"dispenser registered: %s", dispenser_name.data());
  }

  void PseudoDispenser::init_workcell()
  {
    std::function<void (const std::string &)> func =
      [this](const std::string & robot_id)
      {
        // TODO: Debug segmentation fault when calling RCLCPP_INFO here
        RCLCPP_INFO(
          this->node->get_logger(),
          "Dispensing from %s to robot {%s}", this->dispenser_name.data(), robot_id.data()
        );
      };

    dispenser_trigger =
      workcell_triggers::DispenserTrigger::make_dispenser_trigger(node, func);

    RCLCPP_INFO(
      node->get_logger(),
      "Created dispenser node: %s", dispenser_name.data()
    );
  }

} // namespace pseudo_workcells

int main(int argc, char* argv[])
{
    // consider creating a lifecycle node here just to extract
    // params instead of the constructor of the pseudo workcell
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("pseudo_dispenser");
    auto workcell = std::make_shared<pseudo_workcells::PseudoDispenser>(node);
    workcell->init_workcell();
    rclcpp::spin(workcell->node);
    rclcpp::shutdown();
    return 0;
}
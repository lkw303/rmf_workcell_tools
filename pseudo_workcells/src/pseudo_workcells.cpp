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

#include "pseudo_workcells/pseudo_workcells.hpp"


namespace pseudo_workcells{
  PseudoWorkcells::PseudoWorkcells(const rclcpp::Node::SharedPtr _node)
  {
    node = std::move(_node);
    node->declare_parameter("ingestors", std::vector<std::string> ({"product_dropoff_2"}));
    node->declare_parameter("dispensers", std::vector<std::string> ({"storage_1_dispenser"}));

    ingestor_names = node->get_parameter("ingestors").as_string_array();
    dispenser_names = node->get_parameter("dispensers").as_string_array();
    for (auto& i: ingestor_names) {
      RCLCPP_INFO(node->get_logger(),"ingestor: %s", i.data());
    }
    for (auto& d: dispenser_names) {
      RCLCPP_INFO(node->get_logger(),"dispenser: %s", d.data()); 
    }
  }

  void PseudoWorkcells::init_workcells()
  {
    // add this to executor
    executor.add_node(node);

    // create diepsners
    for (auto & d : dispenser_names) {
      auto _n = std::make_shared<rclcpp::Node>(d, d);
      std::function<void (const std::string &)> func =
        [&d, &_n](const std::string & robot_id)
        {
          // TODO: Debug segmentation fault when calling RCLCPP_INFO here

          // RCLCPP_INFO(
          //   _n->get_logger(), 
          //   "Dispensing from %s to robot {%s}", d.data(), robot_id.data()
          // );
        };
      auto dispenser_trigger = 
        workcell_triggers::DispenserTrigger::make_dispenser_trigger(_n, func);

      executor.add_node(_n);
      dispensers.push_back(dispenser_trigger);
      RCLCPP_INFO(
        _n->get_logger(),
        "Created dispenser node: %s", d.data()
      );
    }

    // create ingestors
    for (auto & i : ingestor_names) {
      auto _n = std::make_shared<rclcpp::Node>(i, i);
      std::function<void (const std::string &)> func = 
        [&i, &_n](const std::string & robot_id)
        {
          // TODO: Debug segmentation fault when calling RCLCPP_INFO here
          // RCLCPP_INFO(
          //   _n->get_logger(), 
          //   "Ingesting from robot {%s} to %s", robot_id.data(), i.data()
          // );
        };
      auto ingestor_trigger =
        workcell_triggers::IngestorTrigger::make_ingestor_trigger(_n, func);

      executor.add_node(_n);
      ingestors.push_back(ingestor_trigger);
      RCLCPP_INFO(
        _n->get_logger(), 
        "Created ingestor node: %s", i.data()
      );
    }
  }

  /// Spin executor
  void PseudoWorkcells::spin_all()
  {
    executor.spin();
  }

} // namespace pseudo_workcells
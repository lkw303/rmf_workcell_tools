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

#ifndef WORKCELL_TRIGGERS__DISPENSER_TRIGGER_HPP_
#define WORKCELL_TRIGGERS__DISPENSER_TRIGGER_HPP_

#include <string>
#include <unordered_set>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rmf_dispenser_msgs/msg/dispenser_request.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_result.hpp"
#include "rmf_dispenser_msgs/msg/dispenser_state.hpp"

#include "rmf_task_msgs/msg/dispatch_states.hpp"


namespace workcell_triggers
{

class DispenserTrigger
{
public:
  using DispenserState = rmf_dispenser_msgs::msg::DispenserState;
  using DispenserRequest = rmf_dispenser_msgs::msg::DispenserRequest;
  using DispenserResult = rmf_dispenser_msgs::msg::DispenserResult;
  using Event = std::function<void (const std::string &)>;

  /// Constructor
  explicit DispenserTrigger(const rclcpp::Node::SharedPtr node_);

  virtual ~DispenserTrigger() = default;

  /// Factory method
  static std::shared_ptr<DispenserTrigger> 
    make_dispenser_trigger(const rclcpp::Node::SharedPtr node_, Event event_);

protected:
  /// GUID (aka name) of the dispenser/ingestor
  std::string guid_;

  std::string task_guid_;
  std::unordered_set<std::string> past_task_guids_;

  std::string robot_id_;

private:
  Event event;

  std::atomic_bool lookup_flag_;

  rclcpp::Node::SharedPtr node_;

  /// Dispenser request callback group
  rclcpp::CallbackGroup::SharedPtr dispenser_cb_group_;

  /// Ingesto request callback group
  rclcpp::CallbackGroup::SharedPtr dispatch_state_cb_group_;

  /// Dispenser request subscriber
  rclcpp::Subscription<DispenserRequest>::SharedPtr dispenser_request_sub_;

  /// Dispenser result publisher
  rclcpp::Publisher<DispenserResult>::SharedPtr dispenser_result_pub_;

  /// Dispenser request subscriber
  rclcpp::Subscription<rmf_task_msgs::msg::DispatchStates>::SharedPtr dispatch_state_listener_;

  /// Dispenser request callback function
  void dispenser_request_cb(const DispenserRequest::SharedPtr _request);

  /// Dispatcher state callback function
  void dispatch_state_cb(const rmf_task_msgs::msg::DispatchStates::SharedPtr _request);

  // Add dispenser callback function
  void add_event(Event _event)
  {
    event = std::move(_event);
  }

};

}  // namespace workcell_triggers

#endif  // WORKCELL_TRIGGERS__DISPENSER_TRIGGER_HPP_

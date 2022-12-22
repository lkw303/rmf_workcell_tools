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

#include "rmf_workcell_manager/dispenser_trigger.hpp"

namespace rmf_workcell_manager
{

DispenserTrigger::DispenserTrigger(const rclcpp::Node::SharedPtr node)
{
  node_ = node;

  // Use node name as dispenser/ingestor GUID
  guid_ = node_->get_name();

  lookup_flag_ = false;

  // Create 2 callback groups to run two threads, one for
  // dispenser, one for ingestor.
  dispenser_cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  dispatch_state_cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto dispenser_sub_opt = rclcpp::SubscriptionOptions();
  dispenser_sub_opt.callback_group = dispenser_cb_group_;
  auto dispatch_state_sub_opt = rclcpp::SubscriptionOptions();
  dispatch_state_sub_opt.callback_group = dispatch_state_cb_group_;

  // Initialize dispenser request subscriber
  dispenser_request_sub_ = node_->create_subscription<DispenserRequest>(
    "dispenser_requests",
    // Use default QoS profile
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &DispenserTrigger::dispenser_request_cb,
      this,
      std::placeholders::_1),
    dispenser_sub_opt);

  // Initialize dispenser result publisher
  dispenser_result_pub_ = node_->create_publisher<DispenserResult>(
    "dispenser_results", 10);

  dispatch_state_listener_ = node_->create_subscription<rmf_task_msgs::msg::DispatchStates>(
    "dispatch_states",
    // Use default QoS profile
    rclcpp::SystemDefaultsQoS(),
    std::bind(
      &DispenserTrigger::dispatch_state_cb,
      this,
      std::placeholders::_1),
    dispatch_state_sub_opt);
}

std::shared_ptr<DispenserTrigger> 
  DispenserTrigger::make_dispenser_trigger(const rclcpp::Node::SharedPtr node_, DispenserTrigger::Event event_)
{
  auto dispenser_trigger_ptr = std::make_shared<DispenserTrigger>(node_);
  dispenser_trigger_ptr->add_event(event_);
  return dispenser_trigger_ptr;
}

void DispenserTrigger::dispenser_request_cb(
  const DispenserRequest::SharedPtr _request)
{
  // Only response to the correct guid
  if (_request->target_guid == guid_) {
    if (past_task_guids_.find(_request->request_guid) != past_task_guids_.end()) {
      RCLCPP_WARN(
        node_->get_logger(),
        "This dispenser request has already been processed (id: %s)",
        _request->request_guid.c_str());
      return;
    }
    past_task_guids_.insert(_request->request_guid);
    DispenserResult response;
    response.request_guid = _request->request_guid;

    // Send ACKNOWLEDGED response
    RCLCPP_INFO(
      node_->get_logger(),
      "Dummy dispenser request acknowledged (id: %s)",
      _request->request_guid.c_str());
    response.time = node_->now();
    response.status = DispenserResult::ACKNOWLEDGED;
    dispenser_result_pub_->publish(response);

    task_guid_ = _request->request_guid;

    lookup_flag_ = true;

    int retry = 0;
    while (lookup_flag_ && retry < 3) {
      // Sleep for 1s
      rclcpp::sleep_for(
        std::chrono::seconds(1));
      retry++;
    }

    if (robot_id_.empty()) {
      // Send FAILED response
      RCLCPP_INFO(
        node_->get_logger(),
        "Dispenser request failed (id: %s), cannot find corresponding robot",
        _request->request_guid.c_str());

      rclcpp::sleep_for(
        std::chrono::seconds(1));
      response.time = node_->now();
      response.status = DispenserResult::FAILED;
      dispenser_result_pub_->publish(response);
    } else {
      // Send SUCCESS response
      RCLCPP_INFO(
        node_->get_logger(),
        "Dispenser dispenser request (id: %s), find robot_id (%s)",
        _request->request_guid.c_str(),
        robot_id_.c_str());

      // May need to start a new thread
      event(robot_id_);

      response.time = node_->now();
      response.status = DispenserResult::SUCCESS;
      dispenser_result_pub_->publish(response);
    }

    // Debug check thread id
    RCLCPP_DEBUG(
      node_->get_logger(),
      "Dispenser thread ID: %s",
      std::to_string(
        std::hash<std::thread::id>()(std::this_thread::get_id())
      ).c_str());
  }
}

void DispenserTrigger::dispatch_state_cb(
  const rmf_task_msgs::msg::DispatchStates::SharedPtr dispatch_states_)
{
  if (lookup_flag_) {
    for (auto & state : dispatch_states_->active) {
      if (state.task_id == task_guid_) {
        robot_id_ = state.assignment.expected_robot_name;
        lookup_flag_ = false;
        return;
      }
    }
  }
}
}  // namespace rmf_workcell_manager

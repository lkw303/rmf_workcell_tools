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

#ifndef RMF_WORKCELL_MANAGER__INGESTOR_TRIGGER_HPP_
#define RMF_WORKCELL_MANAGER__INGESTOR_TRIGGER_HPP_

#include <string>
#include <unordered_set>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rmf_ingestor_msgs/msg/ingestor_request.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_result.hpp"
#include "rmf_ingestor_msgs/msg/ingestor_state.hpp"

#include "rmf_task_msgs/msg/dispatch_states.hpp"


namespace rmf_workcell_manager
{

class IngestorTrigger
{
public:
  using IngestorState = rmf_ingestor_msgs::msg::IngestorState;
  using IngestorRequest = rmf_ingestor_msgs::msg::IngestorRequest;
  using IngestorResult = rmf_ingestor_msgs::msg::IngestorResult;
  using Event = std::function<void (const std::string &)>;

  /// Constructor
  explicit IngestorTrigger(const rclcpp::Node::SharedPtr node_);

  virtual ~IngestorTrigger() = default;

  /// Factory method
  static std::shared_ptr<IngestorTrigger> 
    make_ingestor_trigger(const rclcpp::Node::SharedPtr node_, Event event_);

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
  rclcpp::CallbackGroup::SharedPtr ingestor_cb_group_;

  /// Ingesto request callback group
  rclcpp::CallbackGroup::SharedPtr dispatch_state_cb_group_;

  /// Dispenser request subscriber
  rclcpp::Subscription<IngestorRequest>::SharedPtr ingestor_request_sub_;

  /// Dispenser result publisher
  rclcpp::Publisher<IngestorResult>::SharedPtr ingestor_result_pub_;

  /// Dispenser request subscriber
  rclcpp::Subscription<rmf_task_msgs::msg::DispatchStates>::SharedPtr dispatch_state_listener_;

  /// Dispenser request callback function
  void ingestor_request_cb(const IngestorRequest::SharedPtr _request);

  /// Dispatcher state callback function
  void dispatch_state_cb(const rmf_task_msgs::msg::DispatchStates::SharedPtr _request);
  
  /// Add ingestor callback function
  void add_event(Event _event)
  {
    event = std::move(_event);
  }

};

}  // namespace rmf_workcell_manager

#endif  // RMF_WORKCELL_MANAGER__INGESTOR_TRIGGER_HPP_

// Copyright 2020 Gergely Kovács
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

#ifndef ROS2_CORTEX__CORTEXCLIENTNODE_HPP_
#define ROS2_CORTEX__CORTEXCLIENTNODE_HPP_

#include <condition_variable>
#include <functional>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "Cortex.h"
#include "ros2_cortex/CortexClient.hpp"
#include "kroshu_ros2_core/Parameter.hpp"
#include "kroshu_ros2_core/ROS2BaseNode.hpp"
#include "cortex_interfaces/srv/cortex_request_int.hpp"
#include "cortex_interfaces/srv/cortex_request_float.hpp"
#include "cortex_interfaces/srv/cortex_request_empty.hpp"
#include "cortex_interfaces/srv/cortex_request_empty_with_arg.hpp"
#include "cortex_interfaces/srv/cortex_request_fod.hpp"

namespace ros2_cortex
{

class CortexClientNode : public kroshu_ros2_core::ROS2BaseNode
{
public:
  explicit CortexClientNode(const std::string & node_name);
  virtual ~CortexClientNode();
  virtual void dataHandlerFunc_(sFrameOfData & frame_of_data);
  virtual void errorMsgHandlerFunc_(
    CortexVerbosityLevel log_level,
    const std::string & log_message);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

private:
  void setServices();
  void setIntServices();
  void setFloatServices();
  void setEmptyServices();
  void setEmptyWithArgServices();
  bool onOutputFilenameChangeRequest(kroshu_ros2_core::Parameter<std::string> param);
  bool onPlayModeChangeRequest(kroshu_ros2_core::Parameter<std::string> param);

  rclcpp::Service<cortex_interfaces::srv::CortexRequestInt>::SharedPtr post_get_play_mode_service;
  rclcpp::Service<cortex_interfaces::srv::CortexRequestInt>::SharedPtr
    get_context_analog_bit_depth_service;
  rclcpp::Service<cortex_interfaces::srv::CortexRequestInt>::SharedPtr get_up_axis_service;

  rclcpp::Service<cortex_interfaces::srv::CortexRequestFloat>::SharedPtr
    get_context_frame_rate_service;
  rclcpp::Service<cortex_interfaces::srv::CortexRequestFloat>::SharedPtr
    get_context_analog_sample_rate_service;
  rclcpp::Service<cortex_interfaces::srv::CortexRequestFloat>::SharedPtr
    get_conversion_to_millimeters_service;

  rclcpp::Service<cortex_interfaces::srv::CortexRequestEmpty>::SharedPtr start_recording_service;
  rclcpp::Service<cortex_interfaces::srv::CortexRequestEmpty>::SharedPtr stop_recording_service;

  rclcpp::Service<cortex_interfaces::srv::CortexRequestEmptyWithArg>::SharedPtr reset_ids_service;

  rclcpp::Service<cortex_interfaces::srv::CortexRequestFod>::SharedPtr get_frame_of_data_service;

protected:
  std::shared_ptr<CortexClient> cortex_client_ = CortexClient::getInstance();
  sFrameOfData current_fod_;
};
}  // namespace ros2_cortex

#endif  // ROS2_CORTEX__CORTEXCLIENTNODE_HPP_

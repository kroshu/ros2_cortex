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

#include <string>
#include <vector>

#include "ros2_cortex/ROS2BaseNode.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_cortex
{

ROS2BaseNode::ROS2BaseNode(const std::string & node_name)
: rclcpp_lifecycle::LifecycleNode(node_name)
{
}

ROS2BaseNode::~ROS2BaseNode()
{
  ROS2BaseNode::on_shutdown(get_current_state());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseNode::on_configure(const rclcpp_lifecycle::State & state)
{
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result = SUCCESS;
  switch (state.id()) {
    case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
      result = this->on_deactivate(get_current_state());
      if (result != SUCCESS) {
        break;
      }
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
      result = this->on_cleanup(get_current_state());
      break;
    case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
      break;
    default:
      break;
  }
  return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseNode::on_activate(const rclcpp_lifecycle::State & state)
{
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BaseNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "An error occured");
  return SUCCESS;
}

rcl_interfaces::msg::SetParametersResult ROS2BaseNode::onParamChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const rclcpp::Parameter & param : parameters) {
    if (!canSetParameter(param)) {
      RCLCPP_ERROR(this->get_logger(), "Can't set parameter %s in current state",
        param.get_name().c_str());
    } else {
      auto func_it = on_param_change_functions_.find(param.get_name());
      if (func_it == on_param_change_functions_.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid parameter name %s",
          param.get_name().c_str());
      } else {
        result.successful = func_it->second(param);
      }
    }
  }
  return result;
}

bool ROS2BaseNode::canSetParameter(const rclcpp::Parameter & param)
{
  try {
    if (!parameter_set_access_rights_.at(
        param.get_name()).isSetAllowed(this->get_current_state().id()))
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter %s cannot be changed while in state %s",
        param.get_name().c_str(), this->get_current_state().label().c_str());
      return false;
    }
  } catch (const std::out_of_range & e) {
    RCLCPP_ERROR(this->get_logger(),
      "Parameter set access rights for parameter %s couldn't be determined",
      param.get_name().c_str());
    return false;
  }
  return true;
}

void ROS2BaseNode::declareParameter(
  const std::string & name, const rclcpp::ParameterValue & value,
  const ParameterSetAccessRights & rights,
  std::function<bool(const rclcpp::Parameter &)> on_change_callback)
{
  this->declare_parameter(name, value);
  parameter_set_access_rights_.emplace(name, rights);
  on_param_change_functions_.emplace(name, on_change_callback);
}

}  // namespace ros2_cortex
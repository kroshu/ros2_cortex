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

#include <ros2_cortex/CortexClientNode.hpp>
#include <chrono>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <fstream>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "kroshu_ros2_core/ROS2BaseNode.hpp"
#include "ros2_cortex/CortexClient.hpp"

namespace ros2_cortex
{

CortexClientNode::CortexClientNode(const std::string & node_name)
: kroshu_ros2_core::ROS2BaseNode(node_name),
  play_mode_(std::make_shared<Parameter<std::string>>(
      "play_mode", "live",
      ParameterSetAccessRights {
    true, true, true, false},
      [this](const std::string & new_value) {
        return this->onPlayModeChangeRequest(new_value);
      }, *this)),
  output_filename_(
    std::make_shared<Parameter<std::string>>(
      "output_filename", "CortexDefaultOutput.cap",
      ParameterSetAccessRights {
    true, true, false, false},
      [this](const std::string & new_value) {
        return this->onOutputFilenameChangeRequest(new_value);
      }, *this))
{
  ROS2BaseNode::registerParameter(play_mode_);
  ROS2BaseNode::registerParameter(output_filename_);

  using namespace std::placeholders;
  cortex_client_->setDataHandlerFunc(
    [this](sFrameOfData & fod) {
      CortexClientNode::dataHandlerFunc_(fod);
    });
  cortex_client_->setErrorMsgHandlerFunc(
    [this](CortexVerbosityLevel log_level,
    const std::string & log_message) {
      CortexClientNode::errorMsgHandlerFunc_(log_level, log_message);
    });

  param_callback = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters)
    {return onParamChange(parameters);});
}

CortexClientNode::~CortexClientNode()
{
  cortex_client_->freeFrame(current_fod_);
  cortex_client_->exit();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClientNode::on_configure(const rclcpp_lifecycle::State & state)
{
  const std::string empty_str = "";
  cortex_client_->initialize(empty_str, empty_str);
  setServices();
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClientNode::on_activate(const rclcpp_lifecycle::State & state)
{
  std::string play_mode_str = "live";
  bool success = play_mode_->getValue(play_mode_str);
  if (!success) {
    return kroshu_ros2_core::ROS2BaseNode::ERROR;
  }
  onPlayModeChangeRequest(play_mode_str);
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClientNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  std::string play_mode_str = "live";
  bool success = play_mode_->getValue(play_mode_str);
  if (!success) {
    return kroshu_ros2_core::ROS2BaseNode::ERROR;
  }
  if (play_mode_str == "live") {
    cortex_client_->pause();
  } else {
    cortex_client_->postPause();
  }
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

void CortexClientNode::setServices()
{
  setIntServices();
  setFloatServices();
  setEmptyServices();
  setEmptyWithArgServices();

  get_frame_of_data_service =
    this->create_service<cortex_interfaces::srv::CortexRequestFod>(
    "get_frame_of_data", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestFod::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestFod::Response> response) {
      sFrameOfData fod;
      this->cortex_client_->getFrameOfData(fod, request->base_positions);
      response->fod.frame_id = fod.iFrame;
      response->fod.num_of_ui_markers = fod.nUnidentifiedMarkers;
    });
}

void CortexClientNode::setIntServices()
{
  post_get_play_mode_service =
    this->create_service<cortex_interfaces::srv::CortexRequestInt>(
    "post_get_play_mode", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestInt::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestInt::Response> response) {
      response->response = this->cortex_client_->postGetPlayMode();
    });

  get_context_analog_bit_depth_service =
    this->create_service<cortex_interfaces::srv::CortexRequestInt>(
    "get_context_analog_bit_depth", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestInt::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestInt::Response> response) {
      response->response = this->cortex_client_->getContextAnalogBitDepth();
    });

  get_up_axis_service =
    this->create_service<cortex_interfaces::srv::CortexRequestInt>(
    "get_up_axis", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestInt::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestInt::Response> response) {
      response->response = this->cortex_client_->getUpAxis();
    });
}

void CortexClientNode::setFloatServices()
{
  get_context_frame_rate_service =
    this->create_service<cortex_interfaces::srv::CortexRequestFloat>(
    "get_context_frame_rate", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestFloat::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestFloat::Response> response) {
      response->response = this->cortex_client_->getContextFrameRate();
    });

  get_context_analog_sample_rate_service =
    this->create_service<cortex_interfaces::srv::CortexRequestFloat>(
    "get_context_analog_sample_rate", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestFloat::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestFloat::Response> response) {
      response->response = this->cortex_client_->getContextAnalogSampleRate();
    });

  get_conversion_to_millimeters_service =
    this->create_service<cortex_interfaces::srv::CortexRequestFloat>(
    "get_conversion_to_millimeters", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestFloat::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestFloat::Response> response) {
      response->response = this->cortex_client_->getConversionToMillimeters();
    });
}

void CortexClientNode::setEmptyServices()
{
  start_recording_service =
    this->create_service<cortex_interfaces::srv::CortexRequestEmpty>(
    "start_recording", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestEmpty::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestEmpty::Response> response) {
      this->cortex_client_->startRecording();
    });

  stop_recording_service =
    this->create_service<cortex_interfaces::srv::CortexRequestEmpty>(
    "stop_recording", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestEmpty::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestEmpty::Response> response) {
      this->cortex_client_->stopRecording();
    });
}

void CortexClientNode::setEmptyWithArgServices()
{
  reset_ids_service =
    this->create_service<cortex_interfaces::srv::CortexRequestEmptyWithArg>(
    "reset_ids", [this](
      const std::shared_ptr<cortex_interfaces::srv::CortexRequestEmptyWithArg::Request> request,
      std::shared_ptr<cortex_interfaces::srv::CortexRequestEmptyWithArg::Response> response) {
      this->cortex_client_->resetIds(request->arg);
    });
}

bool CortexClientNode::onOutputFilenameChangeRequest(
  const std::string & new_value) const
{
  cortex_client_->setOutputName(new_value);
  return true;
}

bool CortexClientNode::onPlayModeChangeRequest(
  const std::string & new_value) const
{
  if (new_value == "live") {cortex_client_->liveMode();} else if (new_value == "post_forward") {
    cortex_client_->postForward();
  } else if (new_value == "post_backward") {cortex_client_->postBackward();} else {
    RCLCPP_ERROR(get_logger(), "Invalid parameter value for play_mode");
    return false;
  }
  return true;
}

void CortexClientNode::dataHandlerFunc_(sFrameOfData & fod)
{
  cortex_client_->copyFrame(fod, current_fod_);
  RCLCPP_INFO(get_logger(), "Frame " + std::to_string(current_fod_.iFrame));
  RCLCPP_INFO(
    get_logger(),
    "Number of unidentified markers " + std::to_string(current_fod_.nUnidentifiedMarkers));
}

void CortexClientNode::errorMsgHandlerFunc_(
  CortexVerbosityLevel log_level,
  const std::string & log_message)
{
  switch (log_level) {
    case CortexVerbosityLevel::Error:
      RCLCPP_ERROR(get_logger(), log_message);
      break;
    case CortexVerbosityLevel::Warning:
      RCLCPP_WARN(get_logger(), log_message);
      break;
    case CortexVerbosityLevel::Info:
      RCLCPP_INFO(get_logger(), log_message);
      break;
    case CortexVerbosityLevel::Debug:
      RCLCPP_DEBUG(get_logger(), log_message);
      break;
    default:
      break;
  }
}

}  // namespace ros2_cortex

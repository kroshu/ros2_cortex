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
#include "kroshu_ros2_core/ROS2BaseLCNode.hpp"
#include "ros2_cortex/CortexClient.hpp"

namespace ros2_cortex
{

CortexClientNode::CortexClientNode(const std::string & node_name)
: kroshu_ros2_core::ROS2BaseLCNode(node_name)
{
  registerParameter<std::string>(
    "play_mode", "live", kroshu_ros2_core::ParameterSetAccessRights {true, true,
      false, false}, [this](const std::string & new_value) {
      return this->onPlayModeChangeRequest(new_value);
    });
  registerParameter<std::string>(
    "output_filename", "CortexDefaultOutput.cap",
    kroshu_ros2_core::ParameterSetAccessRights {
      true, true, false, false},
    [this](const std::string & new_value) {
      return this->onOutputFilenameChangeRequest(new_value);
    });

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

  param_callback_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return getParameterHandler().onParamChange(parameters);
    });
}

CortexClientNode::~CortexClientNode()
{
  cortex_client_->freeFrame(current_fod_);
  cortex_client_->exit();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClientNode::on_configure(const rclcpp_lifecycle::State &)
{
  const std::string empty_str = "";
  cortex_client_->initialize(empty_str, empty_str);
  setServices();
  return kroshu_ros2_core::ROS2BaseLCNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClientNode::on_activate(const rclcpp_lifecycle::State &)
{
  onPlayModeChangeRequest(play_mode_);
  return kroshu_ros2_core::ROS2BaseLCNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClientNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (play_mode_ == "live") {
    cortex_client_->pause();
  } else {
    cortex_client_->postPause();
  }
  return kroshu_ros2_core::ROS2BaseLCNode::SUCCESS;
}

void CortexClientNode::setServices()
{
  setIntServices();
  setFloatServices();
  setEmptyServices();
  setEmptyWithArgServices();

  get_frame_of_data_service =
    this->create_service<ros2_cortex::srv::CortexRequestFod>(
    "get_frame_of_data", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestFod::Request> request,
      std::shared_ptr<ros2_cortex::srv::CortexRequestFod::Response> response) {
      sFrameOfData fod;
      this->cortex_client_->getFrameOfData(fod, request->base_positions);
      response->fod.frame_id = fod.iFrame;
      response->fod.num_of_ui_markers = fod.nUnidentifiedMarkers;
    });
}

void CortexClientNode::setIntServices()
{
  post_get_play_mode_service =
    this->create_service<ros2_cortex::srv::CortexRequestInt>(
    "post_get_play_mode", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestInt::Request>,
      std::shared_ptr<ros2_cortex::srv::CortexRequestInt::Response> response) {
      response->response = this->cortex_client_->postGetPlayMode();
    });

  get_context_analog_bit_depth_service =
    this->create_service<ros2_cortex::srv::CortexRequestInt>(
    "get_context_analog_bit_depth", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestInt::Request>,
      std::shared_ptr<ros2_cortex::srv::CortexRequestInt::Response> response) {
      response->response = this->cortex_client_->getContextAnalogBitDepth();
    });

  get_up_axis_service =
    this->create_service<ros2_cortex::srv::CortexRequestInt>(
    "get_up_axis", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestInt::Request>,
      std::shared_ptr<ros2_cortex::srv::CortexRequestInt::Response> response) {
      response->response = this->cortex_client_->getUpAxis();
    });
}

void CortexClientNode::setFloatServices()
{
  get_context_frame_rate_service =
    this->create_service<ros2_cortex::srv::CortexRequestFloat>(
    "get_context_frame_rate", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestFloat::Request>,
      std::shared_ptr<ros2_cortex::srv::CortexRequestFloat::Response> response) {
      response->response = this->cortex_client_->getContextFrameRate();
    });

  get_context_analog_sample_rate_service =
    this->create_service<ros2_cortex::srv::CortexRequestFloat>(
    "get_context_analog_sample_rate", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestFloat::Request>,
      std::shared_ptr<ros2_cortex::srv::CortexRequestFloat::Response> response) {
      response->response = this->cortex_client_->getContextAnalogSampleRate();
    });

  get_conversion_to_millimeters_service =
    this->create_service<ros2_cortex::srv::CortexRequestFloat>(
    "get_conversion_to_millimeters", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestFloat::Request>,
      std::shared_ptr<ros2_cortex::srv::CortexRequestFloat::Response> response) {
      response->response = this->cortex_client_->getConversionToMillimeters();
    });
}

void CortexClientNode::setEmptyServices()
{
  start_recording_service =
    this->create_service<ros2_cortex::srv::CortexRequestEmpty>(
    "start_recording", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestEmpty::Request>,
      std::shared_ptr<ros2_cortex::srv::CortexRequestEmpty::Response>) {
      this->cortex_client_->startRecording();
    });

  stop_recording_service =
    this->create_service<ros2_cortex::srv::CortexRequestEmpty>(
    "stop_recording", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestEmpty::Request>,
      std::shared_ptr<ros2_cortex::srv::CortexRequestEmpty::Response>) {
      this->cortex_client_->stopRecording();
    });
}

void CortexClientNode::setEmptyWithArgServices()
{
  reset_ids_service =
    this->create_service<ros2_cortex::srv::CortexRequestEmptyWithArg>(
    "reset_ids", [this](
      const std::shared_ptr<ros2_cortex::srv::CortexRequestEmptyWithArg::Request> request,
      std::shared_ptr<ros2_cortex::srv::CortexRequestEmptyWithArg::Response>) {
      this->cortex_client_->resetIds(request->arg);
    });
}

bool CortexClientNode::onOutputFilenameChangeRequest(
  const std::string & new_value)
{
  cortex_client_->setOutputName(new_value);
  output_filename_ = new_value;
  return true;
}

bool CortexClientNode::onPlayModeChangeRequest(
  const std::string & new_value)
{
  if (new_value == "live") {cortex_client_->liveMode();} else if (new_value == "post_forward") {
    cortex_client_->postForward();
  } else if (new_value == "post_backward") {cortex_client_->postBackward();} else {
    RCLCPP_ERROR(get_logger(), "Invalid parameter value for play_mode");
    return false;
  }
  play_mode_ = new_value;
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

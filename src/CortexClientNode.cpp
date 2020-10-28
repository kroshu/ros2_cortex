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
#include "kroshu_ros2_core/Parameter.hpp"
#include "kroshu_ros2_core/ROS2BaseNode.hpp"
#include "ros2_cortex/CortexClient.hpp"

namespace ros2_cortex
{

std::string getFileExtension(const std::string & file_name)
{
  if (file_name.find_last_of(".") != std::string::npos) {
    return file_name.substr(file_name.find_last_of(".") + 1);
  }
  return "";
}

CortexClientNode::CortexClientNode(const std::string & node_name)
: kroshu_ros2_core::ROS2BaseNode(node_name)
{
  using namespace std::placeholders;
  CortexClient::getInstance().setDataHandlerFunc(
    std::bind(&CortexClientNode::dataHandlerFunc_, this, _1));
  CortexClient::getInstance().setErrorMsgHandlerFunc(
    std::bind(&CortexClientNode::errorMsgHandlerFunc_, this, _1, _2));

  std::string first_comm = "PostPause";
  kroshu_ros2_core::ROS2BaseNode::declareParameter("request_command",
    rclcpp::ParameterValue(
      first_comm),
    rclcpp::ParameterType::PARAMETER_STRING, kroshu_ros2_core::ParameterSetAccessRights {
      false, false, true, false},
    std::bind(&CortexClientNode::onRequestCommandChanged, this, std::placeholders::_1));

  this->set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter> & parameters)
    {return CortexClientNode::onParamChange(parameters);});
}

CortexClientNode::~CortexClientNode()
{
  if (run_thread.joinable()) {run_thread.join();}
  CortexClient::getInstance().freeFrame(current_fod_);
  CortexClient::getInstance().exit();
}

void CortexClientNode::exit()
{
  CortexClient::getInstance().freeFrame(current_fod_);
  CortexClient::getInstance().exit();
}

void CortexClientNode::run()
{
  const std::string empty_str = "";
  CortexClient::getInstance().initialize(empty_str, empty_str);
  CortexClient::getInstance().request(CortexRequestWithNoReturn::PostForward);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClientNode::on_activate(const rclcpp_lifecycle::State & state)
{
  run_thread = std::thread(&CortexClientNode::run, this);
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClientNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  exit();
  run_thread.join();
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

bool CortexClientNode::onRequestCommandChanged(const kroshu_ros2_core::Parameter & param)
{
  CortexReturn success = CortexReturn::Okay;
  const std::string comm_str = param.getValue().get<std::string>();
  if (comm_str == "PostGetPlayMode" || comm_str == "GetContextAnalogBitDepth" ||
    comm_str == "GetUpAxis")
  {
    int response;
    auto command = std::find_if(
      names_of_reqs_with_int_return.begin(),
      names_of_reqs_with_int_return.end(),
      [&comm_str](const auto & temp_comm) {return temp_comm.second == comm_str;});
    success = CortexClient::getInstance().request(command->first, response);
    RCLCPP_INFO(get_logger(),
      "Result of request " + comm_str + ": " + std::to_string(response));
  } else {  // Formatted this way, so that both cpplint and uncrustify can pass
    if (comm_str == "GetContextFrameRate" ||
      comm_str == "GetContextAnalogSampleRate" ||
      comm_str == "GetConversionToMillimeters")
    {
      float response;
      auto command = std::find_if(
        names_of_reqs_with_float_return.begin(),
        names_of_reqs_with_float_return.end(),
        [&comm_str](const auto & temp_comm) {return temp_comm.second == comm_str;});
      success = CortexClient::getInstance().request(command->first, response);
      RCLCPP_INFO(get_logger(),
        "Result of request " + comm_str + ": " + std::to_string(response));
    } else if (comm_str == "GetFrameOfData") {
      sFrameOfData fod;
      success = CortexClient::getInstance().requestFrameOfData(fod, false);
      RCLCPP_INFO(get_logger(), "Frame " + std::to_string(fod.iFrame));
      RCLCPP_INFO(get_logger(),
        "Number of unidentified markers " + std::to_string(fod.nUnidentifiedMarkers));
    } else {
      auto command = std::find_if(
        names_of_reqs_with_no_return.begin(),
        names_of_reqs_with_no_return.end(),
        [&comm_str](const auto & temp_comm) {return temp_comm.second == comm_str;});
      success = CortexClient::getInstance().request(command->first);
    }
  }

  return success == CortexReturn::Okay;
}

void CortexClientNode::dataHandlerFunc_(sFrameOfData & fod)
{
  CortexClient::getInstance().copyFrame(fod, current_fod_);
  RCLCPP_INFO(get_logger(), "Frame " + std::to_string(current_fod_.iFrame));
  RCLCPP_INFO(get_logger(),
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

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

#include <chrono>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include <thread>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "ros2_cortex/CortexClient.hpp"
#include "kroshu_ros2_core/Parameter.hpp"
#include "kroshu_ros2_core/ROS2BaseNode.hpp"

namespace ros2_cortex
{

template<typename Ret, typename ... Params>
struct CortexClient::Callback<Ret(Params...)>
{
  template<typename ... Args>
  static Ret callback(Args... args)
  {
    return func(args ...);
  }
  static std::function<Ret(Params...)> func;
};

template<typename Ret, typename ... Params>
std::function<Ret(Params...)> CortexClient::Callback<Ret(Params...)>::func;

std::string getFileExtension(const std::string & file_name)
{
  if (file_name.find_last_of(".") != std::string::npos) {
    return file_name.substr(file_name.find_last_of(".") + 1);
  }
  return "";
}

void CortexClient::setHandlerFuncs()
{
  Callback<void(sFrameOfData *)>::func = std::bind(&CortexClient::dataHandlerFunc_, this,
      std::placeholders::_1);
  auto data_func =
    static_cast<data_callback_t>(Callback<void(sFrameOfData *)>::callback);
  setDataHandlerFunc(data_func);

  Callback<void(int i_level, char * sz_msg)>::func = std::bind(
    &CortexClient::errorMsgHandlerFunc_, this, std::placeholders::_1,
    std::placeholders::_2);
  auto error_msg_func = static_cast<error_msg__callback_t>(
    Callback<void(int i_level, char * sz_msg)>::callback);
  setErrorMsgHandlerFunc(error_msg_func);
}

CortexClient::CortexClient(const std::string & node_name)
: kroshu_ros2_core::ROS2BaseNode(node_name),
  cortex_mock_(capture_file_path_)
{
  setHandlerFuncs();

  kroshu_ros2_core::ROS2BaseNode::declareParameter("capture_file_path",
    rclcpp::ParameterValue(
      capture_file_path_),
    rclcpp::ParameterType::PARAMETER_STRING, kroshu_ros2_core::ParameterSetAccessRights {
      true, false, false, false},
    std::bind(&CortexClient::onCapFileNameChangeRequest, this, std::placeholders::_1));
  std::string forw_comm = "PostForward";
  kroshu_ros2_core::ROS2BaseNode::declareParameter("request_command",
    rclcpp::ParameterValue(
      forw_comm),
    rclcpp::ParameterType::PARAMETER_STRING, kroshu_ros2_core::ParameterSetAccessRights {
      false, false, true, false},
    std::bind(&CortexClient::onRequestCommandChanged, this, std::placeholders::_1));

  this->set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter> & parameters)
    {return CortexClient::onParamChange(parameters);});
}

CortexClient::~CortexClient()
{
  if (run_thread.joinable()) {run_thread.join();}
  cortex_mock_.freeFrame(&current_fod_);
  cortex_mock_.exit();
}

void CortexClient::exit()
{
  cortex_mock_.freeFrame(&current_fod_);
  cortex_mock_.exit();
}

void CortexClient::run()
{
  cortex_mock_.initialize(nullptr, nullptr);
  std::string req_comm = this->get_parameter("request_command").as_string();
  cortex_mock_.request(&req_comm[0], nullptr, nullptr);
}

int CortexClient::setDataHandlerFunc(void (* dataHandlerFunc)(sFrameOfData * p_frame_of_data))
{
  return cortex_mock_.setDataHandlerFunc(dataHandlerFunc);
}

int CortexClient::setErrorMsgHandlerFunc(
  void (* errorMsgHandlerFunc)(int i_log_level, char * sz_log_message))
{
  return cortex_mock_.setErrorMsgHandlerFunc(errorMsgHandlerFunc);
}

int CortexClient::copyFrame(const sFrameOfData * p_src, sFrameOfData * p_dst) const
{
  return cortex_mock_.copyFrame(p_src, p_dst);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_configure(const rclcpp_lifecycle::State & state)
{
  if (cap_file_path_changed) {
    cortex_mock_ = CortexMock(capture_file_path_);
    cap_file_path_changed = false;
  }
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_activate(const rclcpp_lifecycle::State & state)
{
  run_thread = std::thread(&CortexClient::run, this);
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_deactivate(const rclcpp_lifecycle::State & state)
{
  exit();
  run_thread.join();
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

bool CortexClient::onCapFileNameChangeRequest(const kroshu_ros2_core::Parameter & param)
{
  if (getFileExtension(param.getValue().get<std::string>()) != "json") {
    RCLCPP_ERROR(this->get_logger(), "Invalid file format for parameter %s",
      param.getName().c_str());
    return false;
  }
  std::string file_path = param.getValue().get<std::string>();
  std::ifstream f(file_path.c_str());
  if (!f.good()) {
    RCLCPP_ERROR(this->get_logger(), "File %s doesn't exist or not accessible",
      param.getName().c_str());
    return false;
  }
  capture_file_path_ = file_path;
  cap_file_path_changed = true;
  return true;
}

bool CortexClient::onRequestCommandChanged(const kroshu_ros2_core::Parameter & param)
{
  std::string req_comm = param.getValue().get<std::string>();
  void * p_response = nullptr;
  cortex_mock_.request(&req_comm[0], &p_response, nullptr);
  if (req_comm == "PostGetPlayMode" || req_comm == "GetContextAnalogBitDepth" ||
    req_comm == "GetUpAxis")
  {
    RCLCPP_INFO(get_logger(),
      "Result of request " + req_comm + ": " + std::to_string(*static_cast<int *>(p_response)));
  } else {  // Formatted this way, so that both cpplint and uncrustify can pass
    if (req_comm == "GetContextFrameRate" ||
      req_comm == "GetContextAnalogSampleRate" ||
      req_comm == "GetConversionToMillimeters")
    {
      RCLCPP_INFO(get_logger(),
        "Result of request " + req_comm + ": " + std::to_string(*static_cast<float *>(p_response)));
    } else if (req_comm == "GetFrameOfData") {
      sFrameOfData fod;
      cortex_mock_.copyFrame(static_cast<sFrameOfData *>(p_response), &fod);
      RCLCPP_INFO(get_logger(), "Frame " + std::to_string(fod.iFrame));
      RCLCPP_INFO(get_logger(),
        "Number of unidentified markers " + std::to_string(fod.nUnidentifiedMarkers));
    }
  }

  return true;
}

void CortexClient::dataHandlerFunc_(sFrameOfData * fod)
{
  cortex_mock_.copyFrame(fod, &current_fod_);
  RCLCPP_INFO(get_logger(), "Frame " + std::to_string(current_fod_.iFrame));
  RCLCPP_INFO(get_logger(),
    "Number of unidentified markers " + std::to_string(current_fod_.nUnidentifiedMarkers));
}

void CortexClient::errorMsgHandlerFunc_(int i_level, char * error_msg)
{
  switch (i_level) {
    case 1:
      RCLCPP_ERROR(get_logger(), static_cast<std::string>(error_msg));
      break;
    case 2:
      RCLCPP_WARN(get_logger(), static_cast<std::string>(error_msg));
      break;
    case 3:
      RCLCPP_INFO(get_logger(), static_cast<std::string>(error_msg));
      break;
    case 4:
      RCLCPP_DEBUG(get_logger(), static_cast<std::string>(error_msg));
      break;
    default:
      break;
  }
}

}  // namespace ros2_cortex

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ros2_cortex::CortexClient>("cortex_client");
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}

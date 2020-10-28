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

#ifndef ROS2_CORTEX__CORTEXCLIENT_HPP_
#define ROS2_CORTEX__CORTEXCLIENT_HPP_

#include <vector>
#include <functional>
#include <map>
#include <array>
#include <string>

#include "Cortex.h"
#include "ros2_cortex/CortexClient.hpp"

namespace ros2_cortex
{

enum class CortexReturn
{
  Okay = 0,
  GeneralError,
  ApiError,
  NetworkError,
  TimeOut,
  MemoryError,
  Unrecognized
};

enum class CortexVerbosityLevel
{
  None = 0,
  Error,
  Warning,
  Info,
  Debug
};

enum class CortexThreadPriority
{
  Default,
  Lowest,
  BelowNormal,
  Normal,
  AboveNormal,
  Highest
};

enum class CortexRequestWithNoReturn
{
  LiveMode,
  Pause,
  SetOutputName,
  StartRecording,
  StopRecording,
  ResetIDs,
  PostForward,
  PostBackward,
  PostPause
};

const std::map<CortexRequestWithNoReturn, std::string> names_of_reqs_with_no_return =
{{CortexRequestWithNoReturn::LiveMode, "LiveMode"},
  {CortexRequestWithNoReturn::Pause, "Pause"},
  {CortexRequestWithNoReturn::SetOutputName, "SetOutputName"},
  {CortexRequestWithNoReturn::StartRecording, "StartRecording"},
  {CortexRequestWithNoReturn::StopRecording, "StopRecording"},
  {CortexRequestWithNoReturn::ResetIDs, "ResetIDs"},
  {CortexRequestWithNoReturn::PostForward, "PostForward"},
  {CortexRequestWithNoReturn::PostBackward, "PostBackward"},
  {CortexRequestWithNoReturn::PostPause, "PostPause"}};

enum class CortexRequestWithIntReturn
{
  PostGetPlayMode,
  GetContextAnalogBitDepth,
  GetUpAxis
};

const std::map<CortexRequestWithIntReturn, std::string> names_of_reqs_with_int_return =
{{CortexRequestWithIntReturn::PostGetPlayMode, "PostGetPlayMode"},
  {CortexRequestWithIntReturn::GetContextAnalogBitDepth, "GetContextAnalogBitDepth"},
  {CortexRequestWithIntReturn::GetUpAxis, "GetUpAxis"}};

enum class CortexRequestWithFloatReturn
{
  GetContextFrameRate,
  GetContextAnalogSampleRate,
  GetConversionToMillimeters
};

const std::map<CortexRequestWithFloatReturn, std::string> names_of_reqs_with_float_return =
{{CortexRequestWithFloatReturn::GetContextFrameRate, "GetContextFrameRate"},
  {CortexRequestWithFloatReturn::GetContextAnalogSampleRate, "GetContextAnalogSampleRate"},
  {CortexRequestWithFloatReturn::GetConversionToMillimeters, "GetConversionToMillimeters"}};

class CortexClient
{
public:
  static CortexClient & getInstance();
  CortexClient(CortexClient const &) = delete;
  void operator=(CortexClient const &) = delete;
  CortexReturn getSdkVersion(std::vector<int> & version_nums_placeholder);
  CortexReturn setVerbosityLevel(CortexVerbosityLevel i_level);
  CortexVerbosityLevel getVerbosityLevel();
  CortexReturn setMinTimeout(int ms_timeout);
  int getMinTimeout();
  CortexReturn setErrorMsgHandlerFunc(
    std::function<void(CortexVerbosityLevel,
    const std::string &)> errorMsgHandlerFunc);
  void callDataHandler(sFrameOfData & fod);
  void callErrorMsgHandler(
    CortexVerbosityLevel verb_level,
    const std::string & msg);
  CortexReturn setDataHandlerFunc(
    std::function<void(sFrameOfData &)> dataHandlerFunc);
  CortexReturn sendDataToClients(sFrameOfData & frame_of_data);
  void setClientCommunicationEnabled(bool is_enabled);
  bool isClientCommunicationEnabled();
  void setThreadPriorities(
    CortexThreadPriority listen_for_host, CortexThreadPriority listen_for_data,
    CortexThreadPriority listen_for_clients);
  CortexReturn configurePortNumbers(
    int talk_to_host_port,    // 0 == find available
    int host_port,
    int host_multicast_port,
    int talk_to_clients_request_port = 0,    // 0 == find available
    int talk_to_clients_multicast_port = 0,    // 0 == find available
    int clients_multicast_port = -1);
  CortexReturn initialize(
    const std::string & talk_to_host_nic_card_address,
    const std::string & host_nic_card_address,
    const std::string & host_multicast_address = "",
    const std::string & talk_to_clients_nic_card_address = "",
    const std::string & clients_multicast_address = "");
  CortexReturn getPortNumbers(
    int & talk_to_host_port,
    int & host_port,
    int & host_multicast_port,
    int & talk_to_clients_request_port,
    int & talk_to_clients_multicast_port,
    int & clients_multicast_port);
  CortexReturn getAddresses(
    std::string & talk_to_host_nic_card_address_ph,
    std::string & host_nic_card_address_ph,
    std::string & host_multicast_address_ph,
    std::string & talk_to_clients_nic_card_address_ph,
    std::string & clients_multicast_address_ph);
  CortexReturn getHostInfo(sHostInfo & host_info_ph);
  CortexReturn exit();
  CortexReturn request(
    CortexRequestWithNoReturn command,
    const std::string & optional_arg = "");
  CortexReturn request(CortexRequestWithIntReturn command, int & ret_placeholder);
  CortexReturn request(CortexRequestWithFloatReturn command, float & ret_placeholder);
  CortexReturn requestFrameOfData(sFrameOfData & ret_placeholder, bool base_positions);
  sSkyReturn & skyCommand(const std::string & command, int ms_timeout);
  sBodyDefs & getBodyDefs();
  CortexReturn freeBodyDefs(sBodyDefs & body_defs);
  sFrameOfData & getCurrentFrame();
  CortexReturn copyFrame(const sFrameOfData & src, sFrameOfData & dst);
  CortexReturn freeFrame(sFrameOfData & frame);
  CortexReturn sendHtr(const sHierarchy & p_hierarchy, const tSegmentData & p_frame);
  CortexReturn setMetered(bool active, float fixed_latency);
  // TODO(Gergely Kovacs) maybe use Eigen?
  void constructRotationMatrix(
    std::array<double, 3> angles, int rotation_order,
    std::array<std::array<double, 3>, 3> matrix);
  void extractEulerAngles(
    std::array<std::array<double, 3>, 3> matrix,
    int rotation_order, std::array<double, 3> angles);

private:
  CortexClient() {}
  static const int num_of_version_parts = 4;
  std::function<void(CortexVerbosityLevel,
    const std::string &)> errorMsgHandlerFunc_;
  std::function<void(sFrameOfData &)> dataHandlerFunc_;
};

}  // namespace ros2_cortex

#endif  // ROS2_CORTEX__CORTEXCLIENT_HPP_

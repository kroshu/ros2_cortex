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
  CortexReturn getSdkVersion(std::vector<int> & version_nums_placeholder) const;
  CortexReturn setVerbosityLevel(CortexVerbosityLevel i_level) const;
  CortexVerbosityLevel getVerbosityLevel() const;
  CortexReturn setMinTimeout(int ms_timeout) const;
  int getMinTimeout() const;
  CortexReturn setErrorMsgHandlerFunc(
    const std::function<void(CortexVerbosityLevel,
    const std::string &)> & errorMsgHandlerFunc);
  static void dataHandlerFuncHelper(sFrameOfData * p_fod);
  static void errorMsgHandlerFuncHelper(int i_level, char * msg);
  CortexReturn setDataHandlerFunc(
    const std::function<void(sFrameOfData &)> & dataHandlerFunc);
  CortexReturn sendDataToClients(sFrameOfData & frame_of_data) const;
  void setClientCommunicationEnabled(bool is_enabled) const;
  bool isClientCommunicationEnabled() const;
  void setThreadPriorities(
    CortexThreadPriority listen_for_host, CortexThreadPriority listen_for_data,
    CortexThreadPriority listen_for_clients) const;
  CortexReturn configurePortNumbers(
    int talk_to_host_port,    // 0 == find available
    int host_port,
    int host_multicast_port,
    int talk_to_clients_request_port = 0,    // 0 == find available
    int talk_to_clients_multicast_port = 0,    // 0 == find available
    int clients_multicast_port = -1) const;
  CortexReturn initialize(
    const std::string & talk_to_host_nic_card_address,
    const std::string & host_nic_card_address,
    const std::string & host_multicast_address = "",
    const std::string & talk_to_clients_nic_card_address = "",
    const std::string & clients_multicast_address = "") const;
  CortexReturn getPortNumbers(
    int & talk_to_host_port,
    int & host_port,
    int & host_multicast_port,
    int & talk_to_clients_request_port,
    int & talk_to_clients_multicast_port,
    int & clients_multicast_port) const;
  CortexReturn getAddresses(
    std::string & talk_to_host_nic_card_address_ph,
    std::string & host_nic_card_address_ph,
    std::string & host_multicast_address_ph,
    std::string & talk_to_clients_nic_card_address_ph,
    std::string & clients_multicast_address_ph) const;
  CortexReturn getHostInfo(sHostInfo & host_info_ph) const;
  CortexReturn exit() const;
  void liveMode() const;
  void pause() const;
  void setOutputName(const std::string & file_name) const;
  void startRecording() const;
  void stopRecording() const;
  void resetIds(const std::string & marker_set_name) const;
  void postForward() const;
  void postBackward() const;
  void postPause() const;
  int postGetPlayMode() const;
  float getContextFrameRate() const;
  float getContextAnalogSampleRate() const;
  int getContextAnalogBitDepth() const;
  int getUpAxis() const;
  float getConversionToMillimeters() const;
  void getFrameOfData(sFrameOfData & ret_placeholder, bool base_positions) const;
  sSkyReturn & skyCommand(const std::string & command, int ms_timeout) const;
  sBodyDefs & getBodyDefs() const;
  CortexReturn freeBodyDefs(sBodyDefs & body_defs) const;
  sFrameOfData & getCurrentFrame() const;
  CortexReturn copyFrame(const sFrameOfData & src, sFrameOfData & dst) const;
  CortexReturn freeFrame(sFrameOfData & frame) const;
  CortexReturn sendHtr(const sHierarchy & p_hierarchy, const tSegmentData & p_frame) const;
  CortexReturn setMetered(bool active, float fixed_latency) const;
  // TODO(Gergely Kovacs) maybe use Eigen?
  void constructRotationMatrix(
    const std::array<double, 3> & angles, int rotation_order,
    const std::array<std::array<double, 3>, 3> & matrix) const;
  void extractEulerAngles(
    const std::array<std::array<double, 3>, 3> & matrix,
    int rotation_order, const std::array<double, 3> & angles) const;

private:
  CortexClient() {}
  static const int num_of_version_parts = 4;
  std::function<void(CortexVerbosityLevel,
    const std::string &)> errorMsgHandlerFunc_;
  std::function<void(sFrameOfData &)> dataHandlerFunc_;
  CortexReturn request(
    CortexRequestWithNoReturn command,
    const std::string & optional_arg = "") const;
  CortexReturn request(CortexRequestWithIntReturn command, int & ret_placeholder) const;
  CortexReturn request(CortexRequestWithFloatReturn command, float & ret_placeholder) const;
};

}  // namespace ros2_cortex

#endif  // ROS2_CORTEX__CORTEXCLIENT_HPP_

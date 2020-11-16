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

#include <vector>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <memory>

#include "ros2_cortex/CortexClient.hpp"

namespace ros2_cortex
{

std::shared_ptr<CortexClient> CortexClient::instance_;
std::mutex CortexClient::instance_mutex_;
bool CortexClient::is_instantiated_ = false;
std::function<void(CortexVerbosityLevel,
  const std::string &)> CortexClient::errorMsgHandlerFunc_;
std::function<void(sFrameOfData &)> CortexClient::dataHandlerFunc_;

std::shared_ptr<CortexClient> CortexClient::getInstance()
{
  std::unique_lock<std::mutex> lock(instance_mutex_,
    std::try_to_lock);
  if (!lock.owns_lock() || is_instantiated_) {
    throw std::runtime_error("Cannot instantiate multiple times");
  } else {
    is_instantiated_ = true;
    return instance_;
  }
}

CortexReturn CortexClient::getSdkVersion(std::vector<int> & version_nums_placeholder) const
{
  unsigned char bridge_array[num_of_version_parts];
  auto return_val = static_cast<CortexReturn>(Cortex_GetSdkVersion(bridge_array));
  version_nums_placeholder.resize(num_of_version_parts, 0);
  for (int i = 0; i < num_of_version_parts; ++i) {
    version_nums_placeholder[i] = static_cast<int>(bridge_array[i]);
  }
  return return_val;
}

CortexReturn CortexClient::setVerbosityLevel(CortexVerbosityLevel i_level) const
{
  return static_cast<CortexReturn>(
    Cortex_SetVerbosityLevel(static_cast<int>(i_level)));
}

CortexVerbosityLevel CortexClient::getVerbosityLevel() const
{
  return static_cast<CortexVerbosityLevel>(Cortex_GetVerbosityLevel());
}

CortexReturn CortexClient::setMinTimeout(int ms_timeout) const
{
  return static_cast<CortexReturn>(Cortex_SetMinTimeout(ms_timeout));
}

int CortexClient::getMinTimeout() const
{
  return Cortex_GetMinTimeout();
}

void CortexClient::errorMsgHandlerFuncHelper(int i_level, char * msg)
{
  const std::string str_msg(msg);
  errorMsgHandlerFunc_(
    static_cast<CortexVerbosityLevel>(i_level),
    str_msg);
}

CortexReturn CortexClient::setErrorMsgHandlerFunc(
  std::function<void(CortexVerbosityLevel,
  const std::string &)> errorMsgHandlerFunc)
{
  errorMsgHandlerFunc_ = errorMsgHandlerFunc;
  return static_cast<CortexReturn>(
    Cortex_SetErrorMsgHandlerFunc(errorMsgHandlerFuncHelper));
}

void CortexClient::dataHandlerFuncHelper(sFrameOfData * p_fod)
{
  dataHandlerFunc_(*p_fod);
}

CortexReturn CortexClient::setDataHandlerFunc(
  std::function<void(sFrameOfData &)> dataHandlerFunc)
{
  dataHandlerFunc_ = dataHandlerFunc;
  return static_cast<CortexReturn>(
    Cortex_SetDataHandlerFunc(dataHandlerFuncHelper));
}

CortexReturn CortexClient::sendDataToClients(sFrameOfData & frame_of_data) const
{
  return static_cast<CortexReturn>(Cortex_SendDataToClients(&frame_of_data));
}

void CortexClient::setClientCommunicationEnabled(bool is_enabled) const
{
  Cortex_SetClientCommunicationEnabled(is_enabled);
}

bool CortexClient::isClientCommunicationEnabled() const
{
  return Cortex_IsClientCommunicationEnabled();
}

void CortexClient::setThreadPriorities(
  CortexThreadPriority listen_for_host, CortexThreadPriority listen_for_data,
  CortexThreadPriority listen_for_clients) const
{
  Cortex_SetThreadPriorities(
    static_cast<maThreadPriority>(listen_for_host),
    static_cast<maThreadPriority>(listen_for_data),
    static_cast<maThreadPriority>(listen_for_clients));
}

CortexReturn CortexClient::configurePortNumbers(
  int talk_to_host_port,
  int host_port,
  int host_multicast_port,
  int talk_to_clients_request_port,
  int talk_to_clients_multicast_port,
  int clients_multicast_port) const
{
  return static_cast<CortexReturn>(Cortex_ConfigurePortNumbers(
           talk_to_host_port, host_port,
           host_multicast_port, talk_to_clients_request_port,
           talk_to_clients_multicast_port,
           clients_multicast_port));
}

CortexReturn CortexClient::initialize(
  const std::string & talk_to_host_nic_card_address,
  const std::string & host_nic_card_address,
  const std::string & host_multicast_address,
  const std::string & talk_to_clients_nic_card_address,
  const std::string & clients_multicast_address) const
{
  // Ask const in param, but copy, because expected to be const,
  // but can only be initialized in form of char *
  std::string param_talk_to_host_nic_card_address = talk_to_host_nic_card_address;
  std::string param_host_nic_card_address = host_nic_card_address;
  std::string param_host_multicast_address = host_multicast_address;
  std::string param_talk_to_clients_nic_card_address = talk_to_clients_nic_card_address;
  std::string param_clients_multicast_address = clients_multicast_address;

  return static_cast<CortexReturn>(Cortex_Initialize(
           &param_talk_to_host_nic_card_address[0],
           &param_host_nic_card_address[0],
           &param_host_multicast_address[0],
           &param_talk_to_clients_nic_card_address[0],
           &param_clients_multicast_address[0]));
}

CortexReturn CortexClient::getPortNumbers(
  int & talk_to_host_port,
  int & host_port,
  int & host_multicast_port,
  int & talk_to_clients_request_port,
  int & talk_to_clients_multicast_port,
  int & clients_multicast_port) const
{
  return static_cast<CortexReturn>(Cortex_GetPortNumbers(
           &talk_to_host_port,
           &host_port, &host_multicast_port,
           &talk_to_clients_request_port,
           &talk_to_clients_multicast_port,
           &clients_multicast_port));
}

CortexReturn CortexClient::getAddresses(
  std::string & talk_to_host_nic_card_address_ph,
  std::string & host_nic_card_address_ph,
  std::string & host_multicast_address_ph,
  std::string & talk_to_clients_nic_card_address_ph,
  std::string & clients_multicast_address_ph) const
{
  return static_cast<CortexReturn>(Cortex_GetAddresses(
           &talk_to_host_nic_card_address_ph[0],
           &host_nic_card_address_ph[0],
           &host_multicast_address_ph[0],
           &talk_to_clients_nic_card_address_ph[0],
           &clients_multicast_address_ph[0]));
}

CortexReturn CortexClient::getHostInfo(sHostInfo & host_info_ph) const
{
  return static_cast<CortexReturn>(Cortex_GetHostInfo(&host_info_ph));
}

CortexReturn CortexClient::exit() const
{
  return static_cast<CortexReturn>(Cortex_Exit());
}

void CortexClient::liveMode() const
{
  request(CortexRequestWithNoReturn::LiveMode);
}

void CortexClient::pause() const
{
  request(CortexRequestWithNoReturn::Pause);
}

void CortexClient::setOutputName(const std::string & file_name) const
{
  request(CortexRequestWithNoReturn::SetOutputName, file_name);
}

void CortexClient::startRecording() const
{
  request(CortexRequestWithNoReturn::StartRecording);
}

void CortexClient::stopRecording() const
{
  request(CortexRequestWithNoReturn::StopRecording);
}

void CortexClient::resetIds(const std::string & marker_set_name) const
{
  request(CortexRequestWithNoReturn::ResetIDs, marker_set_name);
}

void CortexClient::postForward() const
{
  request(CortexRequestWithNoReturn::PostForward);
}

void CortexClient::postBackward() const
{
  request(CortexRequestWithNoReturn::PostBackward);
}

void CortexClient::postPause() const
{
  request(CortexRequestWithNoReturn::PostPause);
}

int CortexClient::postGetPlayMode() const
{
  int ret_val;
  request(CortexRequestWithIntReturn::PostGetPlayMode, ret_val);
  return ret_val;
}

float CortexClient::getContextFrameRate() const
{
  float ret_val;
  request(CortexRequestWithFloatReturn::GetContextFrameRate, ret_val);
  return ret_val;
}

float CortexClient::getContextAnalogSampleRate() const
{
  float ret_val;
  request(CortexRequestWithFloatReturn::GetContextAnalogSampleRate, ret_val);
  return ret_val;
}

int CortexClient::getContextAnalogBitDepth() const
{
  int ret_val;
  request(CortexRequestWithIntReturn::GetContextAnalogBitDepth, ret_val);
  return ret_val;
}

int CortexClient::getUpAxis() const
{
  int ret_val;
  request(CortexRequestWithIntReturn::GetUpAxis, ret_val);
  return ret_val;
}

float CortexClient::getConversionToMillimeters() const
{
  float ret_val;
  request(CortexRequestWithFloatReturn::GetConversionToMillimeters, ret_val);
  return ret_val;
}

void CortexClient::getFrameOfData(
  sFrameOfData & ret_placeholder,
  bool base_positions) const
{
  void * p_response = nullptr;
  int req_size = sizeof(void *);
  std::string req_str = base_positions ? "GetFrameOfData=BasePositions" : "GetFrameOfData";
  Cortex_Request(
    &req_str[0],
    &p_response, &req_size);
  ret_placeholder = *static_cast<sFrameOfData *>(p_response);
}

CortexReturn CortexClient::request(
  CortexRequestWithNoReturn command,
  const std::string & optional_arg) const
{
  std::string req_str = optional_arg.empty() ?
    names_of_reqs_with_no_return.at(command) :
    names_of_reqs_with_no_return.at(command) + "=" + optional_arg;
  return static_cast<CortexReturn>(Cortex_Request(&req_str[0], nullptr, nullptr));
}
CortexReturn CortexClient::request(
  CortexRequestWithIntReturn command,
  int & ret_placeholder) const
{
  void * p_response = nullptr;
  int req_size = sizeof(void *);
  std::string req_str = names_of_reqs_with_int_return.at(command);
  auto ret_value = static_cast<CortexReturn>(Cortex_Request(
      &req_str[0],
      &p_response, &req_size));
  ret_placeholder = *static_cast<int *>(p_response);
  return ret_value;
}
CortexReturn CortexClient::request(
  CortexRequestWithFloatReturn command,
  float & ret_placeholder) const
{
  void * p_response = nullptr;
  int req_size = sizeof(void *);
  std::string req_str = names_of_reqs_with_float_return.at(command);
  auto ret_value = static_cast<CortexReturn>(Cortex_Request(
      &req_str[0],
      &p_response, &req_size));
  ret_placeholder = *static_cast<float *>(p_response);
  return ret_value;
}


sSkyReturn & CortexClient::skyCommand(const std::string & command, int ms_timeout) const
{
  std::string temp_command = command;
  return *Cortex_SkyCommand(&temp_command[0], ms_timeout);
}

sBodyDefs & CortexClient::getBodyDefs() const
{
  return *Cortex_GetBodyDefs();
}

CortexReturn CortexClient::freeBodyDefs(sBodyDefs & body_defs) const
{
  return static_cast<CortexReturn>(Cortex_FreeBodyDefs(&body_defs));
}

sFrameOfData & CortexClient::getCurrentFrame() const
{
  return *Cortex_GetCurrentFrame();
}

CortexReturn CortexClient::copyFrame(const sFrameOfData & src, sFrameOfData & dst) const
{
  return static_cast<CortexReturn>(Cortex_CopyFrame(&src, &dst));
}

CortexReturn CortexClient::freeFrame(sFrameOfData & frame) const
{
  return static_cast<CortexReturn>(Cortex_FreeFrame(&frame));
}

CortexReturn CortexClient::sendHtr(
  const sHierarchy & hierarchy,
  const tSegmentData & segment_data) const
{
  // Ask const in param, but copy, because expected to be const,
  // but can only be initialized in form of pointers
  sHierarchy param_hierarchy = hierarchy;
  tSegmentData param_segment_data = {segment_data[0], segment_data[1],
    segment_data[2], segment_data[3],
    segment_data[4], segment_data[5],
    segment_data[6]};
  return static_cast<CortexReturn>(Cortex_SendHtr(
           &param_hierarchy,
           &param_segment_data));
}

CortexReturn CortexClient::setMetered(bool active, float fixed_latency) const
{
  return static_cast<CortexReturn>(Cortex_SetMetered(active, fixed_latency));
}

// TODO(Gergely Kovacs) maybe use Eigen?
void CortexClient::constructRotationMatrix(
  const std::array<double, 3> & angles, int rotation_order,
  const std::array<std::array<double, 3>, 3> & matrix) const
{
  double param_angles[3] = {angles[0], angles[1], angles[2]};
  double param_matrix[3][3] = {matrix[0][0], matrix[0][1], matrix[0][2],
    matrix[1][0], matrix[1][1], matrix[1][2],
    matrix[2][0], matrix[2][1], matrix[2][2]};
  Cortex_ConstructRotationMatrix(param_angles, rotation_order, param_matrix);
}

void CortexClient::extractEulerAngles(
  const std::array<std::array<double, 3>, 3> & matrix,
  int rotation_order, const std::array<double, 3> & angles) const
{
  double param_angles[3] = {angles[0], angles[1], angles[2]};
  double param_matrix[3][3] = {matrix[0][0], matrix[0][1], matrix[0][2],
    matrix[1][0], matrix[1][1], matrix[1][2],
    matrix[2][0], matrix[2][1], matrix[2][2]};
  Cortex_ExtractEulerAngles(param_matrix, rotation_order, param_angles);
}
}  // namespace ros2_cortex

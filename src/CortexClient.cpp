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

#include "ros2_cortex/CortexClient.hpp"

namespace ros2_cortex
{
  CortexReturn CortexClient::getSdkVersion(std::vector<int>& version_nums_placeholder) const{
    unsigned char bridge_array[num_of_version_parts];
    CortexReturn return_val = Cortex_GetSdkVersion(bridge_array);
    version_nums_placeholder.resize(num_of_version_parts, 0);
    for(int i = 0; i < num_of_version_parts; ++i){
      version_nums_placeholder[i] = static_cast<int>(bridge_array[i]);
    }
    return return_val;
  }

  CortexReturn CortexClient::setVerbosityLevel(CortexVerbosityLevel i_level) const{
    return Cortex_SetVerbosityLevel(static_cast<int>(i_level));
  }

  CortexVerbosityLevel CortexClient::getVerbosityLevel() const{
    return Cortex_GetVerbosityLevel();
  }

  CortexReturn CortexClient::setMinTimeout(int ms_timeout) const{
    return Cortex_SetMinTimeout(ms_timeout);
  }

  int CortexClient::getMinTimeout() const{
    return Cortex_GetMinTimeout();
  }

  CortexReturn CortexClient::setErrorMsgHandlerFunc(
      std::function<void(CortexVerbosityLevel,
                         const std::string&)> errorMsgHandlerFunc) const{
    return Cortex_SetErrorMsgHandlerFunc(errorMsgHandlerFunc);
  }

  CortexReturn CortexClient::setDataHandlerFunc(
      std::function<void(sFrameOfData&)> dataHandlerFunc) const{
    return Cortex_SetDataHandlerFunc(dataHandlerFunc);
  }

  CortexReturn CortexClient::sendDataToClients(sFrameOfData& frame_of_data) const{
    return Cortex_SendDataToClients(&frame_of_data);
  }

  void CortexClient::setClientCommunicationEnabled(bool is_enabled) const{
    Cortex_SetClientCommunicationEnabled(is_enabled);
  }

  bool CortexClient::isClientCommunicationEnabled() const{
    return Cortex_IsClientCommunicationEnabled();
  }

  void CortexClient::setThreadPriorities(
    CortexThreadPriority listen_for_host, CortexThreadPriority listen_for_data,
    CortexThreadPriority listen_for_clients) const{
    Cortex_SetThreadPriorities(static_cast<maThreadPriority>(listen_for_host),
                               static_cast<maThreadPriority>(listen_for_data),
                               static_cast<maThreadPriority>(listen_for_clients));
  }

  CortexReturn CortexClient::configurePortNumbers(
      int talk_to_host_port,  // 0 == find available
      int host_port,
      int host_multicast_port,
      int talk_to_clients_request_port = 0,  // 0 == find available
      int talk_to_clients_multicast_port = 0,  // 0 == find available
      int clients_multicast_port = -1) const{
    return Cortex_ConfigurePortNumbers(talk_to_host_port, host_port,
                                       host_multicast_port, talk_to_clients_request_port,
                                       talk_to_clients_multicast_port,
                                       clients_multicast_port);
  }

  CortexReturn CortexClient::initialize(
      const std::string& talk_to_host_nic_card_address,
      const std::string& host_nic_card_address,
      const std::string& host_multicast_address = nullptr,
      const std::string& talk_to_clients_nic_card_address = nullptr,
      const std::string& clients_multicast_address = nullptr) const{
    // Ask const in param, but copy, because expected to be const,
    // but can only be initialized in form of char *
    std::string param_talk_to_host_nic_card_address = talk_to_host_nic_card_address;
    std::string param_host_nic_card_address = host_nic_card_address;
    std::string param_host_multicast_address = host_multicast_address;
    std::string param_talk_to_clients_nic_card_address = talk_to_clients_nic_card_address;
    std::string param_clients_multicast_address = clients_multicast_address;

    return Cortex_Initialize(&param_talk_to_host_nic_card_address[0],
                             &param_host_nic_card_address[0],
                             &param_host_multicast_address[0],
                             &param_talk_to_clients_nic_card_address[0],
                             &param_clients_multicast_address[0]);
  }

  CortexReturn CortexClient::getPortNumbers(
      int& talk_to_host_port,
      int& host_port,
      int& host_multicast_port,
      int& talk_to_clients_request_port,
      int& talk_to_clients_multicast_port,
      int& clients_multicast_port) const{
    return Cortex_GetPortNumbers(&talk_to_host_port, &host_port, &host_multicast_port,
                                 &talk_to_clients_request_port, &talk_to_clients_multicast_port,
                                 &clients_multicast_port);
  }

  CortexReturn CortexClient::getAddresses(
      std::string& talk_to_host_nic_card_address_ph,
      std::string& host_nic_card_address_ph,
      std::string& host_multicast_address_ph,
      std::string& talk_to_clients_nic_card_address_ph,
      std::string& clients_multicast_address_ph) const{
    return Cortex_GetAddresses(&talk_to_host_nic_card_address_ph[0], &host_nic_card_address_ph[0],
                      &host_multicast_address_ph[0], &talk_to_clients_nic_card_address_ph[0],
                      &clients_multicast_address_ph[0]);
  }

  CortexReturn CortexClient::getHostInfo(sHostInfo& host_info_ph) const{
    return Cortex_GetHostInfo(&host_info_ph);
  }

  CortexReturn CortexClient::exit() const{
    return Cortex_Exit();
  }

  CortexReturn CortexClient::request(const std::string& command, void& pp_response,
                                     int& pn_bytes) const{
    // Ask const in param, but copy, because expected to be const,
    // but can only be initialized in form of char *
    std::string temp_command = command;
    return Cortex_Request(&temp_command[0], &pp_response, &pn_bytes);
  }

  sSkyReturn& CortexClient::skyCommand(const std::string& command, int ms_timeout) const{
    std::string temp_command = command;
    return Cortex_SkyCommand(&temp_command[0], ms_timeout);
  }

  sBodyDefs& CortexClient::getBodyDefs() const{
    return Cortex_GetBodyDefs();
  }

  CortexReturn CortexClient::freeBodyDefs(sBodyDefs& body_defs) const{
    return Cortex_FreeBodyDefs(&body_defs);
  }

  sFrameOfData& CortexClient::getCurrentFrame() const{
    return Cortex_GetCurrentFrame();
  }

  CortexReturn CortexClient::copyFrame(const sFrameOfData& src, sFrameOfData& dst) const{
    return Cortex_CopyFrame(&src, &dst);
  }

  CortexReturn CortexClient::freeFrame(sFrameOfData& frame) const{
    return Cortex_FreeFrame(&frame);
  }

  CortexReturn CortexClient::sendHtr(const sHierarchy& hierarchy,
                                     const tSegmentData& segment_data) const{
    // Ask const in param, but copy, because expected to be const,
    // but can only be initialized in form of pointers
    sHierarchy param_hierarchy = hierarchy;
    tSegmentData param_segment_data = segment_data;
    return Cortex_SendHtr(&param_hierarchy, &param_segment_data);
  }

  CortexReturn CortexClient::setMetered(bool active, float fixed_latency) const{
    return Cortex_SetMetered(active, fixed_latency);
  }

  // TODO(Gergely Kovacs) maybe use Eigen?
  void CortexClient::constructRotationMatrix(std::array<double, 3> angles, int rotation_order,
                               std::array<std::array<double, 3>, 3> matrix) const{
    double param_angles[3] = {angles[0], angles[1], angles[2]};
    double param_matrix[3][3];
    std::copy(std::begin(param_matrix), std::end(param_matrix),
              std::begin(matrix));
    Cortex_ConstructRotationMatrix(param_angles, rotation_order, param_matrix);
  }

  void CortexClient::extractEulerAngles(std::array<std::array<double, 3>, 3> matrix,
                          int rotation_order, std::array<double, 3> angles) const{
    double param_angles[3] = {angles[0], angles[1], angles[2]};
        double param_matrix[3][3];
        std::copy(std::begin(param_matrix), std::end(param_matrix),
                  std::begin(matrix));
    Cortex_ExtractEulerAngles(param_matrix, rotation_order, param_angles);
  }
}  // namespace ros2_cortex

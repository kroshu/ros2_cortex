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

#include "Cortex.h"

namespace ros2_cortex
{

enum class CortexReturn {
  Okay = 0,
  GeneralError,
  ApiError,
  NetworkError,
  TimeOut,
  MemoryError,
  Unrecognized
};

enum class CortexVerbosityLevel{
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

class CortexClient{
public:
  CortexReturn getSdkVersion(std::vector<int>& version_nums_placeholder) const;
  CortexReturn setVerbosityLevel(CortexVerbosityLevel i_level) const;
  CortexVerbosityLevel getVerbosityLevel() const;
  CortexReturn setMinTimeout(int ms_timeout) const;
  int getMinTimeout() const;
  CortexReturn setErrorMsgHandlerFunc(
      std::function<void(CortexVerbosityLevel,
                         const std::string&)> errorMsgHandlerFunc) const;
  CortexReturn setDataHandlerFunc(
      std::function<void(sFrameOfData&)> dataHandlerFunc) const;
  CortexReturn sendDataToClients(sFrameOfData& frame_of_data) const;
  void setClientCommunicationEnabled(bool is_enabled) const;
  bool isClientCommunicationEnabled() const;
  void setThreadPriorities(
    CortexThreadPriority listen_for_host, CortexThreadPriority listen_for_data,
    CortexThreadPriority listen_for_clients) const;
  CortexReturn configurePortNumbers(
      int talk_to_host_port,  // 0 == find available
      int host_port,
      int host_multicast_port,
      int talk_to_clients_request_port = 0,  // 0 == find available
      int talk_to_clients_multicast_port = 0,  // 0 == find available
      int clients_multicast_port = -1) const;
  CortexReturn initialize(
      const std::string& talk_to_host_nic_card_address,
      const std::string& host_nic_card_address,
      const std::string& host_multicast_address = nullptr,
      const std::string& talk_to_clients_nic_card_address = nullptr,
      const std::string& clients_multicast_address = nullptr) const;
  CortexReturn getPortNumbers(
      int& talk_to_host_port,
      int& host_port,
      int& host_multicast_port,
      int& talk_to_clients_request_port,
      int& talk_to_clients_multicast_port,
      int& clients_multicast_port) const;
  CortexReturn getAddresses(
      std::string& talk_to_host_nic_card_address_ph,
      std::string& host_nic_card_address_ph,
      std::string& host_multicast_address_ph,
      std::string& talk_to_clients_nic_card_address_ph,
      std::string& clients_multicast_address_ph) const;
  CortexReturn getHostInfo(sHostInfo& host_info_ph) const;
  CortexReturn exit() const;
  CortexReturn request(const std::string& command, void& pp_response, int& pn_bytes) const;
  sSkyReturn& skyCommand(const std::string& command, int ms_timeout) const;
  sBodyDefs& getBodyDefs() const;
  CortexReturn freeBodyDefs(sBodyDefs& body_defs) const;
  sFrameOfData& getCurrentFrame() const;
  CortexReturn copyFrame(const sFrameOfData& src, sFrameOfData& dst) const;
  CortexReturn freeFrame(sFrameOfData& frame) const;
  CortexReturn sendHtr(const sHierarchy& p_hierarchy, const tSegmentData& p_frame) const;
  CortexReturn setMetered(bool active, float fixed_latency) const;
  // TODO(Gergely Kovacs) maybe use Eigen?
  void constructRotationMatrix(std::array<double, 3> angles, int rotation_order,
                               std::array<std::array<double, 3>, 3> matrix) const;
  void extractEulerAngles(std::array<std::array<double, 3>, 3> matrix,
                          int rotation_order, std::array<double, 3> angles) const;
private:
  static const int num_of_version_parts = 4;
};

}  // namespace ros2_cortex

#endif /* ROS2_CORTEX__CORTEXCLIENT_HPP_ */

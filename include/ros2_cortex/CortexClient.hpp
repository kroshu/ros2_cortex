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

#include <condition_variable>
#include <functional>
#include <string>

#include "CortexMock.hpp"

namespace ros2_cortex
{

class CortexClient
{
public:
  explicit CortexClient(const std::string & capture_file_name);
  void run();
  ~CortexClient();
  virtual void dataHandlerFunc_(sFrameOfData * p_frame_of_data) = 0;
  virtual void errorMsgHandlerFunc_(int i_log_level, char * sz_log_message) = 0;
  int setDataHandlerFunc(void (* dataHandlerFunc)(sFrameOfData * p_frame_of_data));
  int setErrorMsgHandlerFunc(void (* errorMsgHandlerFunc)(int i_log_level, char * sz_log_message));
  int copyFrame(const sFrameOfData * p_src, sFrameOfData * p_dst);

private:
  std::string server_addr_ = "127.0.0.1";

protected:
  CortexMock cortex_mock_;
  sFrameOfData current_fod_;
};

}  // namespace ros2_cortex

#endif  // ROS2_CORTEX__CORTEXCLIENT_HPP_

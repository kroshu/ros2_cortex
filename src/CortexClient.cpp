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
#include <thread>
#include <iostream>
#include <string>

#include "ros2_cortex/CortexClient.hpp"

namespace ros2_cortex
{

CortexClient::CortexClient(const std::string & capture_file_name)
: cortex_mock_(capture_file_name)
{
}

CortexClient::~CortexClient()
{
  cortex_mock_.freeFrame(&current_fod_);
}

void CortexClient::run()
{
  cortex_mock_.initialize(&server_addr_[0], &server_addr_[0]);

  std::string forw_comm = "PostForward", backw_comm = "PostBackward", pause_comm = "PostPause";
  cortex_mock_.request(&forw_comm[0], nullptr, nullptr);
  std::this_thread::sleep_for(std::chrono::seconds(30));

  cortex_mock_.request(&backw_comm[0], nullptr, nullptr);
  std::this_thread::sleep_for(std::chrono::seconds(60));

  cortex_mock_.request(&pause_comm[0], nullptr, nullptr);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  std::string rec_comm = "StartRecording";
  cortex_mock_.request(&rec_comm[0], nullptr, nullptr);
  std::this_thread::sleep_for(std::chrono::seconds(5));

  std::string fps_comm = "GetContextFrameRate";
  void * p_response = nullptr;
  cortex_mock_.request(&fps_comm[0], &p_response, nullptr);
  float frame_rate = *static_cast<float *>(p_response);
  std::cout << "Frame rate: " << frame_rate << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(5));

  cortex_mock_.exit();
}

int CortexClient::setDataHandlerFunc(void (* dataHandlerFunc)(sFrameOfData * p_frame_of_data))
{
  cortex_mock_.setDataHandlerFunc(dataHandlerFunc);
}

int CortexClient::setErrorMsgHandlerFunc(
  void (* errorMsgHandlerFunc)(int i_log_level,
  char * sz_log_message))
{
  cortex_mock_.setErrorMsgHandlerFunc(errorMsgHandlerFunc);
}

int CortexClient::copyFrame(const sFrameOfData * p_src, sFrameOfData * p_dst)
{
  cortex_mock_.copyFrame(p_src, p_dst);
}

}  // namespace ros2_cortex

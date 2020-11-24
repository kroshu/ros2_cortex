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

#ifndef ROS2_CORTEX__CORTEXMOCK_HPP_
#define ROS2_CORTEX__CORTEXMOCK_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <memory>
#include <functional>
#include <map>
#include <utility>
#include <thread>
#include <mutex>

#include "Cortex.h"
#include "rapidjson/document.h"

class CortexMock
{
public:
  explicit CortexMock(const std::string & capture_file_name);
  CortexMock(const CortexMock & other) = delete;
  CortexMock & operator=(CortexMock other) = delete;
  ~CortexMock();
  int n_frames_, current_frame_ind_ = 0,
    verbosity_level_ = 2, analog_bit_depth_ = 16;    // a_b_d_ 12 or 16 usually
  static const int read_buffer_size_ = 65536;
  static constexpr float ms_in_s = 1000.0;
  bool running_ = false;
  std::thread run_thread_;
  std::mutex run_cycle_mutex_;
  float conv_rate_to_mm_ = 1.0, frame_rate_ = 50.0, analog_sample_rate_ = 600.0;
  enum class PostPlayMode {backwards = -1, paused, forwards};
  int post_play_mode_ = static_cast<int>(PostPlayMode::paused);
  int post_starter_frame_ = 0, post_end_frame_;
  int repeat_num_ = 0, actual_repeat_ = 0;
  bool is_in_live_ = true, is_playing_in_live = false;
  bool is_recording_in_live = false;
  enum class Request {LiveMode, Pause, SetOutputName, StartRecording,
    StopRecording, ResetIDs, PostForward, PostBackward, PostPause,
    PostGetPlayMode, GetContextFrameRate, GetContextAnalogSampleRate,
    GetContextAnalogBitDepth, GetUpAxis, GetConversionToMillimeters,
    GetFrameOfData};
  std::map<std::string, Request> map_string_to_request = {
    {"LiveMode", Request::LiveMode},
    {"Pause", Request::Pause},
    {"SetOutputName", Request::SetOutputName},
    {"StartRecording", Request::StartRecording},
    {"StopRecording", Request::StopRecording},
    {"ResetIDs", Request::ResetIDs},
    {"PostForward", Request::PostForward},
    {"PostBackward", Request::PostBackward},
    {"PostPause", Request::PostPause},
    {"PostGetPlayMode", Request::PostGetPlayMode},
    {"GetContextFrameRate", Request::GetContextFrameRate},
    {"GetContextAnalogSampleRate", Request::GetContextAnalogSampleRate},
    {"GetContextAnalogBitDepth", Request::GetContextAnalogBitDepth},
    {"GetUpAxis", Request::GetUpAxis},
    {"GetConversionToMillimeters", Request::GetConversionToMillimeters},
    {"GetFrameOfData", Request::GetFrameOfData}
  };
  enum class Axis {x = 0, y, z};
  int axis_up_ = static_cast<int>(Axis::z);
  in_addr host_machine_address_, host_multicast_address_, talk_to_host_address_,
    talk_to_client_address_, client_multicast_address_;
  std::string capture_file_name_;
  int talk_to_host_port_ = 30000, host_port_ = 30001, host_multicast_port_ = 30002;
  int talk_to_clients_request_port_ = 30003, talk_to_clients_multicast_port_ = 30004,
    clients_multicast_port_ = 30005;
  rapidjson::Document document_;
  sFrameOfData current_frame_ = sFrameOfData();
  sBodyDefs body_defs_ = sBodyDefs();
  std::function<void(sFrameOfData *)> dataHandlerFunc_;
  std::function<void(int iLogLevel, char * szLogMessage)> errorMsgHandlerFunc_;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t_active_;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t_after_run_;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> t_after_sleep_;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> last_t_after_sleep_;
  void run();
  bool runCycle();
  void liveRunCycle();
  void postForwardRunCycle();
  void postBackwardRunCycle();
  void extractFrame(sFrameOfData & fod, int iFrame);
  void extractBodies(sFrameOfData & fod, const rapidjson::Value & parent_frame_json);
  // TODO(Gergely Kovacs) check if passing vector would be better here
  void extractMarkers(tMarkerData * markers, int n_markers, const rapidjson::Value & markers_json);
  void extractAnalogData(sAnalogData & adata, const rapidjson::Value & analog_data_json);
  // TODO(Gergely Kovacs) check if passing vector would be better here
  void extractSegments(
    tSegmentData * segments, int n_segments,
    const rapidjson::Value & segments_json);
  void extractBodyDefs(sBodyDefs & body_defs, const rapidjson::Value & body_defs_json);
  void extractBodyDef(sBodyDef & body_def, const rapidjson::Value & body_def_json);
  void readFile();
  void errorMsgInString(int i_level, std::string & msg) const;
  void freeBodyDef(sBodyDef & p_body_def, int n_an_channels);
  void freeBodyData(sBodyData & body_data);
  void copyBodyData(const sBodyData & src_bd, sBodyData & dst_bd) const;
  void copyAnalogData(const sAnalogData & src_ad, sAnalogData & dst_ad) const;
};
#endif  // ROS2_CORTEX__CORTEXMOCK_HPP_

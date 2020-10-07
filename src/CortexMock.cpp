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
#include <string>
#include <utility>
#include <vector>

#include "ros2_cortex/CortexMock.hpp"
#include "rapidjson/filereadstream.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

namespace ros2_cortex
{

CortexMock::CortexMock(std::string & capture_file_name)
: capture_file_name_(capture_file_name)
{
  initReadFile();
}

void CortexMock::initReadFile()
{
  FILE * fp = fopen(capture_file_name_.data(), "r");

  std::vector<char> read_buffer(read_buffer_size_);
  rapidjson::FileReadStream is(fp, read_buffer.data(), read_buffer_size_);

  document_.ParseStream(is);
  fclose(fp);
  n_frames_ = document_["framesArray"].Size();
  // TODO(Gergely Kovacs) capture bodydefs too for the json files
  // extractBodyDefs(body_defs_, document_["bodyDefs"]);
}

CortexMock::CortexMock(const CortexMock & other)
: verbosity_level_(other.verbosity_level_), analog_bit_depth_(other.analog_bit_depth_),
  conv_rate_to_mm_(other.conv_rate_to_mm_), frame_rate_(other.frame_rate_), analog_sample_rate_(
    other.analog_sample_rate_),
  axis_up_(other.axis_up_), dataHandlerFunc_(other.dataHandlerFunc_), errorMsgHandlerFunc_(
    other.errorMsgHandlerFunc_), capture_file_name_(other.capture_file_name_),
  current_framenum_(other.current_framenum_)
{
  // the result of the copy-constructor isn't active yet, even if the other one was active
  // TODO(Gergely Kovacs) if we connect to client, addresses and ports need to be copied, too
  initReadFile();
}

void CortexMock::swap(CortexMock & other) const noexcept
{
  // E.g. if pointers need to be swapped
}

CortexMock & CortexMock::operator=(CortexMock other)
{
  other.swap(*this);  // Copy-constructor and non-throwing swap
  return *this;
}

CortexMock::~CortexMock()
{
  freeFrame(&current_frame_);
}

void CortexMock::getCaptureFilename(std::string & dest) const
{
  dest = capture_file_name_;
}

int CortexMock::getSdkVersion(unsigned char version[4]) const
{
  version[0] = 0;
  version[1] = 8;
  version[2] = 0;
  version[3] = 0;
  return RC_Okay;
}

int CortexMock::setVerbosityLevel(int i_level)
{
  verbosity_level_ = i_level;
  return RC_Okay;
}

int CortexMock::getVerbosityLevel() const
{
  return verbosity_level_;
}

void CortexMock::errorMsgInString(int i_level, std::string & msg_str) const
{
  if (verbosity_level_ >= i_level) {
    errorMsgHandlerFunc_(i_level, &msg_str[0]);
  }
}

int CortexMock::setMinTimeout(int ms_timeout)
{
  // Timeout has no use here actually,
  // because we don't communicate with the host
  std::string error_msg = "No use of timeout in mock version";
  errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int CortexMock::getMinTimeout()
{
  // Timeout has no use here actually,
  // because we don't communicate with the host
  std::string error_msg = "No use of timeout in mock version";
  errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int CortexMock::setErrorMsgHandlerFunc(
  void (* errorMsgHandlerFunc)(int i_log_level,
  char * sz_log_message))
{
  errorMsgHandlerFunc_ = errorMsgHandlerFunc;
  return RC_Okay;
}

int CortexMock::setDataHandlerFunc(void (* dataHandlerFunc)(sFrameOfData * p_frame_of_data))
{
  dataHandlerFunc_ = dataHandlerFunc;
  return RC_Okay;
}

int CortexMock::sendDataToClients(sFrameOfData * p_frame_of_data)
{
  // TODO(Gergely Kovacs) send through TCP if client communication is going to be enabled
  std::string error_msg = "No communication with client in mock version";
  errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

void CortexMock::setClientCommunicationEnabled(int b_enabled)
{
  // TODO(Gergely Kovacs) do we enable communication with clients in the mock
  std::string error_msg = "No communication with client in mock version";
  errorMsgInString(VL_Error, error_msg);
}

int CortexMock::isClientCommunicationEnabled()
{
  // TODO(Gergely Kovacs) do we enable communication with clients in the mock
  std::string error_msg = "No communication with client in mock version";
  errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

void CortexMock::setThreadPriorities(
  maThreadPriority listen_for_host,
  maThreadPriority listen_for_data,
  maThreadPriority listen_for_clients)
{
  // TODO(Gergely Kovacs) if communicating with client,
  // should we differentiate priority of listening to data and listening for clients?
  std::string error_msg = "No communication with client neither with host in mock version";
  errorMsgInString(VL_Error, error_msg);
}

int CortexMock::configurePortNumbers(
  int talk_to_host_port,
  int host_port,
  int host_multicast_port,
  int talk_to_clients_request_port,
  int talk_to_clients_multicast_port,
  int clients_multicast_port)
{
  // TODO(Gergely Kovacs) needs to
  // be implemented if client communication is going to be enabled
  std::string error_msg = "No communication with client neither with host in mock version";
  errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int CortexMock::initialize(
  char * sz_talk_to_host_nic_card_address,
  char * sz_host_nic_card_address,
  char * sz_host_multicast_address,
  char * sz_talk_to_clients_nic_card_address,
  char * sz_clients_multicast_address)
{
  std::string error_msg = "No communication with client neither with host in mock version";
  errorMsgInString(VL_Warning, error_msg);
  // TODO(Gergely Kovacs) address storing needs to
  // be implemented if client communication is going to be enabled
  // and complete default initialization, too: sz_host_multicast_address = 225.1.1.1,
  // sz_talk_to_clients_nic_card_address = 127.0.0.1, sz_clients_multicast_address = 225.1.1.2

  std::thread run_thread(&CortexMock::run, this);
  return RC_Okay;
}

int CortexMock::getPortNumbers(
  int * talk_to_host_port,
  int * host_port,
  int * host_multicast_port,
  int * talk_to_clients_request_port,
  int * talk_to_clients_multicast_port,
  int * clients_multicast_port)
{
  // TODO(Gergely Kovacs) needs to be implemented if client communication is going to be enabled
  std::string error_msg = "No communication with client neither with host in mock version";
  errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int CortexMock::getAddresses(
  char * sz_talk_to_host_nic_card_address,
  char * sz_host_nic_card_address,
  char * sz_host_multicast_address,
  char * sz_talk_to_clients_nic_card_address,
  char * sz_clients_multicast_address)
{
  // TODO(Gergely Kovacs) needs to be implemented if client communication is going to be enabled
  std::string error_msg = "No communication with client neither with host in mock version";
  errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int CortexMock::getHostInfo(sHostInfo * p_host_info)
{
  // TODO(Gergely Kovacs) needs to be implemented if client communication is going to be enabled
  std::string error_msg = "Found mock version, no communication with host in mock version";
  errorMsgInString(VL_Warning, error_msg);
  return RC_ApiError;
}

int CortexMock::exit()
{
  running_ = false;
  play_mode_ = static_cast<int>(PlayMode::paused);
  return RC_Okay;
}

int CortexMock::request(char * sz_command, void ** pp_response, int * pn_bytes)
{
  std::string command(sz_command), command_extra;
  size_t pos = command.find('=');
  if (pos != std::string::npos) {
    command_extra = command.substr(pos);
    command = command.substr(0, pos);
  }
  auto found_it = map_string_to_request.find(command);
  if (found_it == map_string_to_request.end()) {
    std::string error_msg = "Unrecognized request";
    errorMsgInString(VL_Error, error_msg);
    return RC_Unrecognized;
  }
  std::string error_msg = "Mock handles no live mode requests";
  Request req_type = found_it->second;
  switch (req_type) {
    // Mock doesn't and can't deal with live mode requests
    // TODO(Gergely Kovacs) should it?
    case Request::LiveMode:
      errorMsgInString(VL_Error, error_msg);
      return RC_ApiError;
    case Request::Pause:
      errorMsgInString(VL_Error, error_msg);
      return RC_ApiError;
    case Request::SetOutputName:
      errorMsgInString(VL_Error, error_msg);
      return RC_ApiError;
    case Request::StartRecording:
      errorMsgInString(VL_Error, error_msg);
      return RC_ApiError;
    case Request::StopRecording:
      errorMsgInString(VL_Error, error_msg);
      return RC_ApiError;
    case Request::ResetIDs:
      errorMsgInString(VL_Error, error_msg);
      return RC_ApiError;
    // Mock does deal with post mode requests though
    case Request::PostForward:
      play_mode_ = static_cast<int>(PlayMode::forwards);
      break;
    case Request::PostBackward:
      play_mode_ = static_cast<int>(PlayMode::backwards);
      break;
    case Request::PostPause:
      play_mode_ = static_cast<int>(PlayMode::paused);
      break;
    case Request::PostGetPlayMode:
      *pp_response = &play_mode_;
      break;
    case Request::GetContextFrameRate:
      *pp_response = &frame_rate_;
      break;
    case Request::GetContextAnalogSampleRate:
      *pp_response = &analog_sample_rate_;
      break;
    case Request::GetContextAnalogBitDepth:
      *pp_response = &analog_bit_depth_;
      break;
    case Request::GetUpAxis:
      *pp_response = &axis_up_;
      break;
    case Request::GetConversionToMillimeters:
      *pp_response = &conv_rate_to_mm_;
      break;
    case Request::GetFrameOfData:
      if (command_extra.empty()) {*pp_response = &current_frame_;}
      // TODO(Gergely Kovacs) else return markerset base pos
      break;

    default:
      error_msg = "Unrecognized request";
      errorMsgInString(VL_Error, error_msg);
      return RC_Unrecognized;
  }

  return RC_Okay;
}

sSkyReturn * CortexMock::skyCommand(char * sz_command, int ms_timeout)
{
  // TODO(Gergely Kovacs) implement this function
  std::string error_msg = "Function not implemented yet";
  errorMsgInString(VL_Error, error_msg);
  return nullptr;
}

sBodyDefs * CortexMock::getBodyDefs()
{
  return &body_defs_;
}

int CortexMock::freeBodyDefs(sBodyDefs * p_body_defs)
{
  if (p_body_defs == nullptr) {return RC_MemoryError;}
  int n_body_defs = p_body_defs->nBodyDefs;
  for (int i = 0; i < n_body_defs; ++i) {
    delete[] p_body_defs->BodyDefs[i].szName;

    int n_markers = p_body_defs->nAnalogChannels;
    for (int i_marker_name = 0; i_marker_name < n_markers; ++i_marker_name) {
      if (p_body_defs->BodyDefs[i].szMarkerNames[i_marker_name] != nullptr) {
        delete[] p_body_defs->BodyDefs[i].szMarkerNames[i_marker_name];
      }
    }
    if (n_markers > 0) {delete[] p_body_defs->BodyDefs[i].szMarkerNames;}

    int n_segments = p_body_defs->BodyDefs[i].Hierarchy.nSegments;
    for (int i_segment_name = 0; i_segment_name < n_segments; ++i_segment_name) {
      if (p_body_defs->BodyDefs[i].Hierarchy.szSegmentNames[i_segment_name] != nullptr) {
        delete[] p_body_defs->BodyDefs[i].Hierarchy.szSegmentNames[i_segment_name];
      }
    }
    if (n_segments > 0) {
      delete[] p_body_defs->BodyDefs[i].Hierarchy.szSegmentNames;
      delete[] p_body_defs->BodyDefs[i].Hierarchy.iParents;
    }

    int n_dofs = p_body_defs->nAnalogChannels;
    for (int i_dof_name = 0; i_dof_name < n_dofs; ++i_dof_name) {
      if (p_body_defs->BodyDefs[i].szDofNames[i_dof_name] != nullptr) {
        delete[] p_body_defs->BodyDefs[i].szDofNames[i_dof_name];
      }
    }
    if (n_markers > 0) {delete[] p_body_defs->BodyDefs[i].szDofNames;}
  }


  int n_analogch = p_body_defs->nAnalogChannels;
  for (int i_ach_name = 0; i_ach_name < n_analogch; ++i_ach_name) {
    if (p_body_defs->szAnalogChannelNames[i_ach_name] != nullptr) {
      delete[] p_body_defs->szAnalogChannelNames[i_ach_name];
    }
  }
  if (n_analogch > 0) {delete[] p_body_defs->szAnalogChannelNames;}
  delete[] p_body_defs->AnalogLoVoltage;
  delete[] p_body_defs->AnalogHiVoltage;
  return RC_Okay;
}

sFrameOfData * CortexMock::getCurrentFrame()
{
  extractFrame(current_frame_, current_framenum_);
  return &current_frame_;
}

int CortexMock::copyFrame(const sFrameOfData * p_src, sFrameOfData * p_dst) const
{
  p_dst->iFrame = p_src->iFrame;
  p_dst->fDelay = p_src->fDelay;
  p_dst->nBodies = p_src->nBodies;
  int n_bodies = p_dst->nBodies;
  for (int i = 0; i < n_bodies; i++) {
    memcpy(p_dst->BodyData[i].szName, p_src->BodyData[i].szName,
      strlen(p_src->BodyData[i].szName) + 1);

    p_dst->BodyData[i].nMarkers = p_src->BodyData[i].nMarkers;
    int n_markers = p_dst->BodyData[i].nMarkers;
    if (n_markers > 0) {
      p_dst->BodyData[i].Markers = new tMarkerData[n_markers];
      memcpy(p_dst->BodyData[i].Markers, p_src->BodyData[i].Markers,
        n_markers * sizeof(tMarkerData));
    }
    p_dst->BodyData[i].fAvgMarkerResidual = p_src->BodyData[i].fAvgMarkerResidual;

    p_dst->BodyData[i].nSegments = p_src->BodyData[i].nSegments;
    int n_segments = p_dst->BodyData[i].nSegments;
    if (n_segments > 0) {
      p_dst->BodyData[i].Segments = new tSegmentData[n_segments];
      memcpy(p_dst->BodyData[i].Segments, p_src->BodyData[i].Segments,
        n_segments * sizeof(tSegmentData));
    }

    p_dst->BodyData[i].nDofs = p_src->BodyData[i].nDofs;
    int n_dofs = p_dst->BodyData[i].nDofs;
    if (n_dofs > 0) {
      p_dst->BodyData[i].Dofs = new tDofData[n_dofs];
      memcpy(p_dst->BodyData[i].Dofs, p_src->BodyData[i].Dofs, n_dofs * sizeof(tDofData));
    }
    p_dst->BodyData[i].fAvgDofResidual = p_src->BodyData[i].fAvgDofResidual;
    p_dst->BodyData[i].nIterations = p_src->BodyData[i].nIterations;

    p_dst->BodyData[i].ZoomEncoderValue = p_src->BodyData[i].ZoomEncoderValue;
    p_dst->BodyData[i].FocusEncoderValue = p_src->BodyData[i].FocusEncoderValue;
    p_dst->BodyData[i].IrisEncoderValue = p_src->BodyData[i].IrisEncoderValue;
    memcpy(p_dst->BodyData[i].CamTrackParams, p_src->BodyData[i].CamTrackParams,
      sizeof(tCamTrackParameters));

    p_dst->BodyData[i].nEvents = p_src->BodyData[i].nEvents;
    int n_events = p_dst->BodyData[i].nEvents;
    char ** src_event_str_ptr = p_src->BodyData[i].Events;
    p_dst->BodyData[i].Events = new char *[n_events];
    char ** dst_event_str_ptr = p_dst->BodyData[i].Events;
    // TODO(Gergely Kovacs) test this and other
    // functionalities too which aren't tested by capture files
    for (int j = 0; j < n_events; ++j, ++src_event_str_ptr, ++dst_event_str_ptr) {
      std::string event_str(*src_event_str_ptr);
      *dst_event_str_ptr = new char[event_str.length() + 1];
      memcpy(*dst_event_str_ptr, event_str.data(), strlen(event_str.data()) + 1);
    }
  }

  p_dst->nUnidentifiedMarkers = p_src->nUnidentifiedMarkers;
  int n_ui_markers = p_dst->nUnidentifiedMarkers;
  if (n_ui_markers > 0) {
    p_dst->UnidentifiedMarkers = new tMarkerData[n_ui_markers];
    memcpy(p_dst->UnidentifiedMarkers, p_src->UnidentifiedMarkers,
      n_ui_markers * sizeof(tMarkerData));
  }

  p_dst->AnalogData.nAnalogChannels = p_src->AnalogData.nAnalogChannels;
  p_dst->AnalogData.nAnalogSamples = p_src->AnalogData.nAnalogSamples;
  int n_analogs = p_dst->AnalogData.nAnalogChannels * p_dst->AnalogData.nAnalogSamples;
  if (n_analogs > 0) {
    p_dst->AnalogData.AnalogSamples = new short[n_analogs];  // NOLINT
    memcpy(p_dst->AnalogData.AnalogSamples, p_src->AnalogData.AnalogSamples,
      n_analogs * sizeof(short));  // NOLINT
  }

  p_dst->AnalogData.nForcePlates = p_src->AnalogData.nForcePlates;
  p_dst->AnalogData.nForceSamples = p_src->AnalogData.nForceSamples;
  int n_forces = p_dst->AnalogData.nForcePlates * p_dst->AnalogData.nForceSamples;
  if (n_forces > 0) {
    p_dst->AnalogData.Forces = new tForceData[n_forces];
    memcpy(p_dst->AnalogData.Forces, p_src->AnalogData.Forces, n_forces * sizeof(tForceData));
  }


  p_dst->AnalogData.nAngleEncoders = p_src->AnalogData.nAngleEncoders;
  p_dst->AnalogData.nAngleEncoderSamples = p_src->AnalogData.nAngleEncoderSamples;
  int n_all_ae_samples = p_dst->AnalogData.nAngleEncoders * p_dst->AnalogData.nAngleEncoderSamples;
  if (n_analogs > 0) {
    p_dst->AnalogData.AngleEncoderSamples = new double[n_all_ae_samples];
    memcpy(p_dst->AnalogData.AngleEncoderSamples, p_src->AnalogData.AngleEncoderSamples,
      n_all_ae_samples * sizeof(double));
  }

  p_dst->RecordingStatus.bRecording = p_src->RecordingStatus.bRecording;
  p_dst->RecordingStatus.iFirstFrame = p_src->RecordingStatus.iFirstFrame;
  p_dst->RecordingStatus.iLastFrame = p_src->RecordingStatus.iLastFrame;
  memcpy(p_dst->RecordingStatus.szFilename, p_src->RecordingStatus.szFilename,
    strlen(p_src->RecordingStatus.szFilename) + 1);

  p_dst->TimeCode.iFrames = p_src->TimeCode.iFrames;
  p_dst->TimeCode.iHours = p_src->TimeCode.iHours;
  p_dst->TimeCode.iMinutes = p_src->TimeCode.iMinutes;
  p_dst->TimeCode.iSeconds = p_src->TimeCode.iSeconds;
  p_dst->TimeCode.iStandard = p_src->TimeCode.iStandard;

  return RC_GeneralError;
}

int CortexMock::freeFrame(sFrameOfData * p_frame)
{
  if (p_frame == nullptr) {return RC_MemoryError;}
  int n_bodies = p_frame->nBodies;
  if (n_bodies > 0) {
    for (int i_body = 0; i_body < n_bodies; ++i_body) {
      sBodyData & i_body_data = p_frame->BodyData[i_body];
      if (i_body_data.nMarkers > 0) {delete[] i_body_data.Markers;}
      if (i_body_data.nSegments > 0) {delete[] i_body_data.Segments;}
      if (i_body_data.nDofs > 0) {delete[] i_body_data.Dofs;}
      int n_events = i_body_data.nEvents;
      for (int i_event = 0; i_event < n_events; ++i_event) {
        if (i_body_data.Events[i_event] != nullptr) {delete[] i_body_data.Events[i_event];}
      }
      if (i_body_data.nEvents > 0) {delete[] i_body_data.Events;}
    }
  }

  if (p_frame->nUnidentifiedMarkers > 0) {delete[] p_frame->UnidentifiedMarkers;}
  if (p_frame->AnalogData.nAnalogSamples > 0 && p_frame->AnalogData.nAnalogChannels > 0) {
    delete[] p_frame->AnalogData.AnalogSamples;
  }
  if (p_frame->AnalogData.nForceSamples > 0 && p_frame->AnalogData.nForcePlates > 0) {
    delete[] p_frame->AnalogData.Forces;
  }
  if (p_frame->AnalogData.nAngleEncoderSamples > 0 && p_frame->AnalogData.nAngleEncoders > 0) {
    delete[] p_frame->AnalogData.AngleEncoderSamples;
  }
  return RC_Okay;
}

int CortexMock::sendHtr(sHierarchy * p_hierarchy, tSegmentData * p_frame)
{
  // TODO(Gergely Kovacs) implement this function
  std::string error_msg = "Function not implemented yet";
  errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int CortexMock::setMetered(bool b_active, float f_fixed_latency)
{
  // TODO(Gergely Kovacs) if we enable communication with clients in the mock
  std::string error_msg = "No communication with client in mock version";
  errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

void CortexMock::constructRotationMatrix(
  double angles[3], int i_rotation_order,
  double matrix[3][3])
{
  // TODO(Gergely Kovacs) implement this function
  std::string error_msg = "Function not implemented yet";
  errorMsgInString(VL_Error, error_msg);
}

void CortexMock::extractEulerAngles(double matrix[3][3], int i_rotation_order, double angles[3])
{
  // TODO(Gergely Kovacs) implement this function
  std::string error_msg = "Function not implemented yet";
  errorMsgInString(VL_Error, error_msg);
}

void CortexMock::extractBodyDefs(sBodyDefs & body_defs, const rapidjson::Value & body_defs_json)
{
  body_defs.nBodyDefs = body_defs_json["nBodyDefs"].GetInt();
  int n_body_defs = body_defs.nBodyDefs;
  rapidjson::Value body_def_array(rapidjson::kArrayType);
  for (int i = 0; i < n_body_defs; i++) {
    extractBodyDef(body_defs.BodyDefs[i], body_def_array[i]);
  }

  body_defs.nAnalogChannels = body_defs_json["nAnalogChannels"].GetInt();
  int n_analog_channels = body_defs.nAnalogChannels;
  body_defs.szAnalogChannelNames = new char *[n_analog_channels];
  char ** dst_analogch_names_ptr = body_defs.szAnalogChannelNames;
  for (int i = 0; i < n_analog_channels; ++i, ++dst_analogch_names_ptr) {
    std::string analogch_name = body_defs_json["analogChannelNames"][i].GetString();
    *dst_analogch_names_ptr = new char[analogch_name.length() + 1];
    memcpy(*dst_analogch_names_ptr, analogch_name.data(), strlen(analogch_name.data()) + 1);
  }

  body_defs.nForcePlates = body_defs_json["nForcePlates"].GetInt();
  body_defs.AnalogBitDepth = body_defs_json["analogBitDepth"].GetInt();

  body_defs.AnalogLoVoltage = new float[n_analog_channels];
  body_defs.AnalogHiVoltage = new float[n_analog_channels];
  for (int i = 0; i < n_analog_channels; i++) {
    body_defs.AnalogLoVoltage[i] = body_defs_json["analogLoVoltage"][i].GetFloat();
    body_defs.AnalogHiVoltage[i] = body_defs_json["analogHiVoltage"][i].GetFloat();
  }
}

void CortexMock::extractBodyDef(sBodyDef & body_def, const rapidjson::Value & body_def_json)
{
  std::string name(body_def_json["name"].GetString());
  body_def.szName = new char[name.length() + 1];
  memcpy(body_def.szName, name.data(), strlen(name.data()) + 1);

  body_def.nMarkers = body_def_json["nMarkers"].GetInt();
  int n_markers = body_def.nMarkers;
  body_def.szMarkerNames = new char *[n_markers];
  char ** dst_marker_names_ptr = body_def.szMarkerNames;
  for (int i = 0; i < n_markers; ++i, ++dst_marker_names_ptr) {
    std::string marker_name = body_def_json["markerNames"][i].GetString();
    *dst_marker_names_ptr = new char[marker_name.length() + 1];
    memcpy(*dst_marker_names_ptr, marker_name.data(), strlen(marker_name.data()) + 1);
  }

  body_def.Hierarchy.nSegments = body_def_json["hierarchy"]["nSegments"].GetInt();
  int n_segments = body_def.Hierarchy.nSegments;
  body_def.Hierarchy.szSegmentNames = new char *[n_segments];
  char ** dst_segment_names_ptr = body_def.Hierarchy.szSegmentNames;
  body_def.Hierarchy.iParents = new int[n_segments];
  for (int i = 0; i < n_segments; ++i, ++dst_segment_names_ptr) {
    std::string segment_name = body_def_json["hierarchy"]["segmentNames"][i].GetString();
    *dst_segment_names_ptr = new char[segment_name.length() + 1];
    memcpy(*dst_segment_names_ptr, segment_name.data(), strlen(segment_name.data()) + 1);

    body_def.Hierarchy.iParents[i] = body_def_json["hierarchy"]["parents"][i].GetInt();
  }

  body_def.nDofs = body_def_json["nDofs"].GetInt();
  int n_dofs = body_def.nDofs;
  body_def.szDofNames = new char *[n_dofs];
  char ** dst_dof_names_ptr = body_def.szDofNames;
  for (int i = 0; i < n_dofs; ++i, ++dst_dof_names_ptr) {
    std::string dof_name = body_def_json["dofNames"][i].GetString();
    *dst_dof_names_ptr = new char[dof_name.length() + 1];
    memcpy(*dst_dof_names_ptr, dof_name.data(), strlen(dof_name.data()) + 1);
  }
}

void CortexMock::extractBodies(sFrameOfData & fod, const rapidjson::Value & parent_frame_json)
{
  int n_bodies = fod.nBodies;
  for (int i = 0; i < n_bodies; ++i) {
    const rapidjson::Value & i_body_json = parent_frame_json["bodies"][i];
    sBodyData & i_body_data = fod.BodyData[i];
    memcpy(i_body_data.szName, i_body_json["name"].GetString(),
      strlen(i_body_json["name"].GetString()) + 1);
    i_body_data.nMarkers = i_body_json["nMarkers"].GetInt();
    if (i_body_data.nMarkers > 0) {
      i_body_data.Markers = new tMarkerData[i_body_data.nMarkers];
      extractMarkers(i_body_data.Markers, i_body_data.nMarkers, i_body_json["markers"]);
    }
    i_body_data.fAvgMarkerResidual = i_body_json["fAvgMarkerResidual"].GetFloat();
    i_body_data.nSegments = i_body_json["nSegments"].GetInt();
    if (i_body_data.nSegments > 0) {
      i_body_data.Segments = new tSegmentData[i_body_data.nSegments];
      extractSegments(i_body_data.Segments, i_body_data.nSegments, i_body_json["segments"]);
    }
    i_body_data.nDofs = i_body_json["nDofs"].GetInt();

    int n_dofs = i_body_data.nDofs;
    if (n_dofs > 0) {
      i_body_data.Dofs = new double[n_dofs];
      for (int i_dof = 0; i_dof < n_dofs; ++i_dof) {
        i_body_data.Dofs[i] = i_body_json["dofs"][i].GetDouble();
      }
    }


    i_body_data.fAvgDofResidual = i_body_json["fAvgDofResidual"].GetFloat();
    i_body_data.nIterations = i_body_json["nIterations"].GetInt();
    i_body_data.ZoomEncoderValue = i_body_json["encoderZoom"].GetInt();
    i_body_data.FocusEncoderValue = i_body_json["encoderFocus"].GetInt();
    i_body_data.IrisEncoderValue = i_body_json["encoderIris"].GetInt();

    const rapidjson::Value & cam_track_params = i_body_json["camTrackParams"];
    i_body_data.CamTrackParams[0] = cam_track_params["offsetX"].GetDouble();
    i_body_data.CamTrackParams[1] = cam_track_params["offsetY"].GetDouble();
    i_body_data.CamTrackParams[2] = cam_track_params["offsetZ"].GetDouble();
    i_body_data.CamTrackParams[3] = cam_track_params["offsetAngleX"].GetDouble();
    i_body_data.CamTrackParams[4] = cam_track_params["offsetAngleY"].GetDouble();
    i_body_data.CamTrackParams[5] = cam_track_params["offsetAngleZ"].GetDouble();
    i_body_data.CamTrackParams[6] = cam_track_params["videoWidth"].GetDouble();
    i_body_data.CamTrackParams[7] = cam_track_params["videoHeight"].GetDouble();
    i_body_data.CamTrackParams[8] = cam_track_params["opticalCenterX"].GetDouble();
    i_body_data.CamTrackParams[9] = cam_track_params["opticalCenterY"].GetDouble();
    i_body_data.CamTrackParams[10] = cam_track_params["fovX"].GetDouble();
    i_body_data.CamTrackParams[11] = cam_track_params["fovY"].GetDouble();
    i_body_data.CamTrackParams[12] = cam_track_params["pixelAspect"].GetDouble();
    i_body_data.CamTrackParams[13] = cam_track_params["firstCoefficient"].GetDouble();

    i_body_data.nEvents = i_body_json["nEvents"].GetInt();

    int n_events = i_body_data.nEvents;
    if (n_events > 0) {
      i_body_data.Events = new char *[n_events];
      for (int i_event = 0; i_event < n_events; ++i_event) {
        i_body_data.Events[i_event] = new char[i_body_json["events"][i].GetStringLength()];
        memcpy(i_body_data.Events[i_event], i_body_json["events"][i].GetString(),
          strlen(i_body_json["events"][i].GetString()) + 1);
      }
    }
  }
}

void CortexMock::extractMarkers(
  tMarkerData * markers, int n_markers,
  const rapidjson::Value & markers_json)
{
  for (int i_marker = 0; i_marker < n_markers; ++i_marker) {
    markers[i_marker][0] = markers_json[i_marker]["x"].GetFloat();
    markers[i_marker][1] = markers_json[i_marker]["y"].GetFloat();
    markers[i_marker][2] = markers_json[i_marker]["z"].GetFloat();
  }
}

void CortexMock::extractAnalogData(sAnalogData & adata, const rapidjson::Value & analog_data_json)
{
  adata.nAnalogChannels = analog_data_json["nAnalogChannels"].GetInt();
  int n_channels = adata.nAnalogChannels;
  adata.nAnalogSamples = analog_data_json["nAnalogSamples"].GetInt();
  int n_samples = adata.nAnalogSamples;
  int index = 0;
  if (n_channels > 0 || n_samples > 0) {
    adata.AnalogSamples = new short[n_samples * n_channels];  // NOLINT
    for (int i_sample = 0; i_sample < n_samples; i_sample++) {
      for (int i_channel = 0; i_channel < n_channels; i_channel++) {
        index = i_sample * n_samples + i_channel;
        adata.AnalogSamples[index] =
          static_cast<short>(analog_data_json["analogSamples"][index]["value"].GetInt());  // NOLINT
      }
    }
  }

  adata.nForcePlates = analog_data_json["nForcePlates"].GetInt();
  int n_force_plates = adata.nForcePlates;
  adata.nForceSamples = analog_data_json["nForceSamples"].GetInt();
  int n_force_samples = adata.nForceSamples;
  if (n_force_plates > 0 || n_force_samples > 0) {
    adata.Forces = new tForceData[n_force_samples * n_force_plates];
    for (int i_sample = 0; i_sample < n_force_samples; i_sample++) {
      for (int i_plate = 0; i_plate < n_force_plates; i_plate++) {
        index = i_sample * n_samples + i_plate;
        const rapidjson::Value & force_json = analog_data_json["forces"][index];
        adata.Forces[index][0] = force_json["x"].GetFloat();
        adata.Forces[index][1] = force_json["y"].GetFloat();
        adata.Forces[index][2] = force_json["z"].GetFloat();
        adata.Forces[index][3] = force_json["fX"].GetFloat();
        adata.Forces[index][4] = force_json["fY"].GetFloat();
        adata.Forces[index][5] = force_json["fZ"].GetFloat();
        adata.Forces[index][6] = force_json["mZ"].GetFloat();
      }
    }
  }

  adata.nAngleEncoders = analog_data_json["nAngleEncoders"].GetInt();
  int n_angle_encoders = adata.nAngleEncoders;
  adata.nAngleEncoderSamples =
    analog_data_json["nAngleEncoderSamples"].GetInt();
  int n_angle_encoder_samples = adata.nAngleEncoderSamples;
  if (n_angle_encoders > 0 || n_angle_encoder_samples > 0) {
    adata.AngleEncoderSamples = new double[n_angle_encoders * n_angle_encoder_samples];
    for (int i_sample = 0; i_sample < n_angle_encoder_samples; i_sample++) {
      for (int i_enc = 0; i_enc < n_angle_encoders; i_enc++) {
        index = i_sample * n_angle_encoder_samples + i_enc;
        adata.AngleEncoderSamples[index] =
          analog_data_json["angleEncoderSamples"][index]["value"].GetDouble();
      }
    }
  }
}

void CortexMock::extractSegments(
  tSegmentData * segments, int n_segments,
  const rapidjson::Value & segments_json)
{
  for (int i_segment = 0; i_segment < n_segments; ++i_segment) {
    segments[i_segment][0] = segments_json[i_segment]["x"].GetDouble();
    segments[i_segment][1] = segments_json[i_segment]["y"].GetDouble();
    segments[i_segment][2] = segments_json[i_segment]["z"].GetDouble();
    segments[i_segment][3] = segments_json[i_segment]["aX"].GetDouble();
    segments[i_segment][4] = segments_json[i_segment]["aY"].GetDouble();
    segments[i_segment][5] = segments_json[i_segment]["aZ"].GetDouble();
    segments[i_segment][6] = segments_json[i_segment]["length"].GetDouble();
  }
}

void CortexMock::extractFrame(sFrameOfData & fod, int i_frame)
{
  freeFrame(&fod);
  const rapidjson::Value & frame = document_["framesArray"][i_frame];
  fod.iFrame = frame["frame"].GetInt();
  fod.fDelay = frame["frameDelay"].GetFloat();
  fod.nBodies = frame["nBodies"].GetInt();

  if (fod.nBodies > 0) {extractBodies(fod, frame);}

  fod.nUnidentifiedMarkers = frame["nUnidentifiedMarkers"].GetInt();

  if (fod.nUnidentifiedMarkers > 0) {
    fod.UnidentifiedMarkers = new tMarkerData[fod.nUnidentifiedMarkers];
    extractMarkers(fod.UnidentifiedMarkers, fod.nUnidentifiedMarkers, frame["unidentifiedMarkers"]);
  }

  extractAnalogData(fod.AnalogData, frame["analogData"]);

  const rapidjson::Value & rc_status = frame["recordingStatus"];
  fod.RecordingStatus.bRecording = rc_status["recording"].GetInt();
  fod.RecordingStatus.iFirstFrame = rc_status["firstFrame"].GetInt();
  fod.RecordingStatus.iLastFrame = rc_status["lastFrame"].GetInt();
  memcpy(fod.RecordingStatus.szFilename, rc_status["captureFileName"].GetString(),
    strlen(rc_status["captureFileName"].GetString()) + 1);

  const rapidjson::Value & tc_value = frame["timeCode"];
  fod.TimeCode.iHours = tc_value["hours"].GetInt();
  fod.TimeCode.iMinutes = tc_value["minutes"].GetInt();
  fod.TimeCode.iSeconds = tc_value["seconds"].GetInt();
  fod.TimeCode.iFrames = tc_value["frames"].GetInt();

  // Correct this by changing generation to simple int instead of converting twice
  std::string standard = tc_value["standard"].GetString();
  if (standard == "SMPTE") {
    fod.TimeCode.iStandard = 1;
  } else if (standard == "FILM") {
    fod.TimeCode.iStandard = 2;
  } else if (standard == "EBU") {
    fod.TimeCode.iStandard = 3;
  } else if (standard == "SYSTEMCLOCK") {
    fod.TimeCode.iStandard = 4;
  }
}

void CortexMock::run()
{
  running_ = true;
  while (running_) {
    switch (static_cast<PlayMode>(play_mode_)) {
      case PlayMode::forwards:
        extractFrame(current_frame_, current_framenum_);
        dataHandlerFunc_(&current_frame_);
        current_framenum_ = current_framenum_ < n_frames_ - 1 ? current_framenum_ + 1 : 0;
        break;
      case PlayMode::backwards:
        extractFrame(current_frame_, current_framenum_);
        dataHandlerFunc_(&current_frame_);
        current_framenum_ = current_framenum_ > 0 ? current_framenum_ - 1 : n_frames_ - 1;
        break;

      default:
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(ms_in_s / frame_rate_)));
  }
}

}  // namespace ros2_cortex

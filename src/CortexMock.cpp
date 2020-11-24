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
#include <mutex>
#include <iostream>
#include <iomanip>

#include "rapidjson/filereadstream.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "ros2_cortex/CortexMock.hpp"
#include "Cortex.h"

CortexMock::CortexMock(const std::string & capture_file_name)
: capture_file_name_(capture_file_name)
{
}

void CortexMock::readFile()
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

CortexMock::~CortexMock()
{
  if (run_thread_.joinable()) {run_thread_.join();}
  // Locking mutex not needed here, run_thread is joint
  Cortex_FreeFrame(&current_frame_);
}

void CortexMock::errorMsgInString(int i_level, std::string & msg_str) const
{
  if (verbosity_level_ >= i_level) {
    errorMsgHandlerFunc_(i_level, &msg_str[0]);
  }
}

void CortexMock::freeBodyDef(sBodyDef & p_body_def, int n_an_channels)
{
  delete[] p_body_def.szName;

  // Free markers
  int n_markers = n_an_channels;
  for (int i_marker_name = 0; i_marker_name < n_markers; ++i_marker_name) {
    if (p_body_def.szMarkerNames[i_marker_name] != nullptr) {
      delete[] p_body_def.szMarkerNames[i_marker_name];
    }
  }
  if (n_markers > 0) {delete[] p_body_def.szMarkerNames;}

  // Free segments
  int n_segments = p_body_def.Hierarchy.nSegments;
  for (int i_segment_name = 0; i_segment_name < n_segments; ++i_segment_name) {
    if (p_body_def.Hierarchy.szSegmentNames[i_segment_name] != nullptr) {
      delete[] p_body_def.Hierarchy.szSegmentNames[i_segment_name];
    }
  }
  if (n_segments > 0) {
    delete[] p_body_def.Hierarchy.szSegmentNames;
    delete[] p_body_def.Hierarchy.iParents;
  }

  // Free dofs
  int n_dofs = n_an_channels;
  for (int i_dof_name = 0; i_dof_name < n_dofs; ++i_dof_name) {
    if (p_body_def.szDofNames[i_dof_name] != nullptr) {
      delete[] p_body_def.szDofNames[i_dof_name];
    }
  }
  if (n_markers > 0) {delete[] p_body_def.szDofNames;}
}

void CortexMock::copyBodyData(const sBodyData & src_bd, sBodyData & dst_bd) const
{
  memcpy(
    dst_bd.szName, src_bd.szName,
    strlen(src_bd.szName) + 1);

  // Copy markers
  dst_bd.nMarkers = src_bd.nMarkers;
  int n_markers = dst_bd.nMarkers;
  if (n_markers > 0) {
    dst_bd.Markers = new tMarkerData[n_markers];
    memcpy(
      dst_bd.Markers, src_bd.Markers,
      n_markers * sizeof(tMarkerData));
  }
  dst_bd.fAvgMarkerResidual = src_bd.fAvgMarkerResidual;

  // Copy segments
  dst_bd.nSegments = src_bd.nSegments;
  int n_segments = dst_bd.nSegments;
  if (n_segments > 0) {
    dst_bd.Segments = new tSegmentData[n_segments];
    memcpy(
      dst_bd.Segments, src_bd.Segments,
      n_segments * sizeof(tSegmentData));
  }

  // Copy dofs
  dst_bd.nDofs = src_bd.nDofs;
  int n_dofs = dst_bd.nDofs;
  if (n_dofs > 0) {
    dst_bd.Dofs = new tDofData[n_dofs];
    memcpy(dst_bd.Dofs, src_bd.Dofs, n_dofs * sizeof(tDofData));
  }
  dst_bd.fAvgDofResidual = src_bd.fAvgDofResidual;
  dst_bd.nIterations = src_bd.nIterations;

  // Copy cam params
  dst_bd.ZoomEncoderValue = src_bd.ZoomEncoderValue;
  dst_bd.FocusEncoderValue = src_bd.FocusEncoderValue;
  dst_bd.IrisEncoderValue = src_bd.IrisEncoderValue;
  memcpy(
    dst_bd.CamTrackParams, src_bd.CamTrackParams,
    sizeof(tCamTrackParameters));

  // Copy events
  dst_bd.nEvents = src_bd.nEvents;
  int n_events = dst_bd.nEvents;
  char ** src_event_str_ptr = src_bd.Events;
  dst_bd.Events = new char *[n_events];
  char ** dst_event_str_ptr = dst_bd.Events;
  // TODO(Gergely Kovacs) test this and other
  // functionalities too which aren't tested by capture files
  for (int j = 0; j < n_events; ++j, ++src_event_str_ptr, ++dst_event_str_ptr) {
    std::string event_str(*src_event_str_ptr);
    *dst_event_str_ptr = new char[event_str.length() + 1];
    memcpy(*dst_event_str_ptr, event_str.data(), strlen(event_str.data()) + 1);
  }
}

void CortexMock::copyAnalogData(const sAnalogData & src_ad, sAnalogData & dst_ad) const
{
  // Copy analog samples
  dst_ad.nAnalogChannels = src_ad.nAnalogChannels;
  dst_ad.nAnalogSamples = src_ad.nAnalogSamples;
  int n_analogs = dst_ad.nAnalogChannels * dst_ad.nAnalogSamples;
  if (n_analogs > 0) {
    dst_ad.AnalogSamples = new short[n_analogs];  // NOLINT
    memcpy(
      dst_ad.AnalogSamples, src_ad.AnalogSamples,
      n_analogs * sizeof(short));  // NOLINT
  }

  // Copy force samples
  dst_ad.nForcePlates = src_ad.nForcePlates;
  dst_ad.nForceSamples = src_ad.nForceSamples;
  int n_forces = dst_ad.nForcePlates * dst_ad.nForceSamples;
  if (n_forces > 0) {
    dst_ad.Forces = new tForceData[n_forces];
    memcpy(dst_ad.Forces, src_ad.Forces, n_forces * sizeof(tForceData));
  }

  // Copy angle encoder samples
  dst_ad.nAngleEncoders = src_ad.nAngleEncoders;
  dst_ad.nAngleEncoderSamples = src_ad.nAngleEncoderSamples;
  int n_all_ae_samples = dst_ad.nAngleEncoders * dst_ad.nAngleEncoderSamples;
  if (n_analogs > 0) {
    dst_ad.AngleEncoderSamples = new double[n_all_ae_samples];
    memcpy(
      dst_ad.AngleEncoderSamples, src_ad.AngleEncoderSamples,
      n_all_ae_samples * sizeof(double));
  }
}

void CortexMock::freeBodyData(sBodyData & body_data)
{
  if (body_data.nMarkers > 0) {delete[] body_data.Markers;}
  if (body_data.nSegments > 0) {delete[] body_data.Segments;}
  if (body_data.nDofs > 0) {delete[] body_data.Dofs;}
  int n_events = body_data.nEvents;
  for (int i_event = 0; i_event < n_events; ++i_event) {
    if (body_data.Events[i_event] != nullptr) {delete[] body_data.Events[i_event];}
  }
  if (body_data.nEvents > 0) {delete[] body_data.Events;}
}

void CortexMock::extractBodyDefs(sBodyDefs & body_defs, const rapidjson::Value & body_defs_json)
{
  // Extract body defs
  body_defs.nBodyDefs = body_defs_json["nBodyDefs"].GetInt();
  int n_body_defs = body_defs.nBodyDefs;
  rapidjson::Value body_def_array(rapidjson::kArrayType);
  for (int i = 0; i < n_body_defs; i++) {
    extractBodyDef(body_defs.BodyDefs[i], body_def_array[i]);
  }

  // Extract analog data
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

  // Extract markers
  body_def.nMarkers = body_def_json["nMarkers"].GetInt();
  int n_markers = body_def.nMarkers;
  body_def.szMarkerNames = new char *[n_markers];
  char ** dst_marker_names_ptr = body_def.szMarkerNames;
  for (int i = 0; i < n_markers; ++i, ++dst_marker_names_ptr) {
    std::string marker_name = body_def_json["markerNames"][i].GetString();
    *dst_marker_names_ptr = new char[marker_name.length() + 1];
    memcpy(*dst_marker_names_ptr, marker_name.data(), strlen(marker_name.data()) + 1);
  }

  // Extract segments
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

  // Extract dofs
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
    memcpy(
      i_body_data.szName, i_body_json["name"].GetString(),
      strlen(i_body_json["name"].GetString()) + 1);

    // Extract markers
    i_body_data.nMarkers = i_body_json["nMarkers"].GetInt();
    if (i_body_data.nMarkers > 0) {
      i_body_data.Markers = new tMarkerData[i_body_data.nMarkers];
      extractMarkers(i_body_data.Markers, i_body_data.nMarkers, i_body_json["markers"]);
    }
    i_body_data.fAvgMarkerResidual = i_body_json["fAvgMarkerResidual"].GetFloat();

    // Extract segments
    i_body_data.nSegments = i_body_json["nSegments"].GetInt();
    if (i_body_data.nSegments > 0) {
      i_body_data.Segments = new tSegmentData[i_body_data.nSegments];
      extractSegments(i_body_data.Segments, i_body_data.nSegments, i_body_json["segments"]);
    }

    // Extract dofs
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

    // Extract cam params
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

    // Extract events
    i_body_data.nEvents = i_body_json["nEvents"].GetInt();
    int n_events = i_body_data.nEvents;
    if (n_events > 0) {
      i_body_data.Events = new char *[n_events];
      for (int i_event = 0; i_event < n_events; ++i_event) {
        i_body_data.Events[i_event] = new char[i_body_json["events"][i].GetStringLength()];
        memcpy(
          i_body_data.Events[i_event], i_body_json["events"][i].GetString(),
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
  // Extract analog samples
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

  // Extract force samples
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

  // Extract angle encoders
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
  Cortex_FreeFrame(&fod);
  const rapidjson::Value & frame = document_["framesArray"][i_frame];
  fod.iFrame = frame["frame"].GetInt();
  fod.fDelay = frame["frameDelay"].GetFloat();

  // Extract body datas
  fod.nBodies = frame["nBodies"].GetInt();
  if (fod.nBodies > 0) {extractBodies(fod, frame);}

  // Extract unidentified markers
  fod.nUnidentifiedMarkers = frame["nUnidentifiedMarkers"].GetInt();
  if (fod.nUnidentifiedMarkers > 0) {
    fod.UnidentifiedMarkers = new tMarkerData[fod.nUnidentifiedMarkers];
    extractMarkers(fod.UnidentifiedMarkers, fod.nUnidentifiedMarkers, frame["unidentifiedMarkers"]);
  }

  extractAnalogData(fod.AnalogData, frame["analogData"]);

  // Extract recording data
  const rapidjson::Value & rc_status = frame["recordingStatus"];
  fod.RecordingStatus.bRecording = rc_status["recording"].GetInt();
  fod.RecordingStatus.iFirstFrame = rc_status["firstFrame"].GetInt();
  fod.RecordingStatus.iLastFrame = rc_status["lastFrame"].GetInt();
  memcpy(
    fod.RecordingStatus.szFilename, rc_status["captureFileName"].GetString(),
    strlen(rc_status["captureFileName"].GetString()) + 1);

  // Extract time data
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

bool CortexMock::runCycle()
{
  std::lock_guard<std::mutex> guard(run_cycle_mutex_);
  if (!running_) {return false;}
  if (is_in_live_) {
    liveRunCycle();
  } else {
    switch (static_cast<PostPlayMode>(post_play_mode_)) {
      case PostPlayMode::forwards:
        postForwardRunCycle();
        break;
      case PostPlayMode::backwards:
        postBackwardRunCycle();
        break;
      default:
        break;
    }
  }
  return true;
}

void CortexMock::liveRunCycle()
{
  if (is_playing_in_live) {
    extractFrame(current_frame_, current_frame_ind_);
    dataHandlerFunc_(&current_frame_);
    if (current_frame_ind_ < n_frames_ - 1) {
      current_frame_ind_++;
    } else {
      current_frame_ind_ = 0;
      if (is_recording_in_live) {repeat_num_++;}
    }
  }
}

void CortexMock::postForwardRunCycle()
{
  extractFrame(current_frame_, current_frame_ind_);
  dataHandlerFunc_(&current_frame_);
  if (actual_repeat_ < repeat_num_) {
    current_frame_ind_ = current_frame_ind_ < n_frames_ - 1 ?
      current_frame_ind_ + 1 : 0;
    if (current_frame_ind_ == 0) {actual_repeat_++;}
  } else {
    if (current_frame_ind_ == post_end_frame_) {actual_repeat_ = 0;}
    current_frame_ind_ = current_frame_ind_ < post_end_frame_ ?
      current_frame_ind_ + 1 : post_starter_frame_;
  }
}

void CortexMock::postBackwardRunCycle()
{
  extractFrame(current_frame_, current_frame_ind_);
  dataHandlerFunc_(&current_frame_);
  if (actual_repeat_ > 0) {
    current_frame_ind_ = current_frame_ind_ > 0 ?
      current_frame_ind_ - 1 : n_frames_ - 1;
    if (current_frame_ind_ == n_frames_ - 1) {actual_repeat_--;}
  } else {
    if (current_frame_ind_ == post_starter_frame_) {actual_repeat_ = repeat_num_;}
    current_frame_ind_ = current_frame_ind_ > post_starter_frame_ ?
      current_frame_ind_ - 1 : post_end_frame_;
  }
}

void CortexMock::run()
{
  running_ = true;
  t_after_sleep_ =
		  std::chrono::time_point_cast<std::chrono::nanoseconds>(
				  std::chrono::system_clock::now());
  last_t_after_sleep_ = t_after_sleep_;
  while (true) {
    // Store time as t_active_
    t_active_ =
  		  std::chrono::time_point_cast<std::chrono::nanoseconds>(
  				  std::chrono::system_clock::now());
    if (!runCycle()) {return;}
    auto t_to_active = t_active_ - t_after_sleep_;
    auto frame_time = std::chrono::nanoseconds(static_cast<int>(ms_in_s / frame_rate_) * 1000000);
    auto time_of_sleep = frame_time - t_to_active;
    // Store time as t_after_run_
    t_after_run_ =
  		  std::chrono::time_point_cast<std::chrono::nanoseconds>(
  				  std::chrono::system_clock::now());
    // Time of sleep: (ms_in_s/frame_rate) - (t_after_run_ - t_active_) - (t_active_ - t_after_sleep_)
    auto t_run = t_after_run_ - t_active_;
    time_of_sleep -= t_run;
    std::this_thread::sleep_for(time_of_sleep);
    // Store time as t_after_sleep_
    t_after_sleep_ =
  		  std::chrono::time_point_cast<std::chrono::nanoseconds>(
  				  std::chrono::system_clock::now());

    std::time_t time_now = std::chrono::system_clock::to_time_t(t_after_sleep_);
    auto after_sleep_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        		t_after_sleep_.time_since_epoch()) % 1000;
    auto after_sleep_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            		t_after_sleep_.time_since_epoch());
    auto after_run_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    			t_after_run_.time_since_epoch()) % 1000;
    auto after_run_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    		t_after_run_.time_since_epoch());
    std::cout << "Time at end of the frame: " <<
    		std::put_time(std::localtime(&time_now), "%H:%M:%S"); // HH:MM:SS
    std::cout << '.' << std::setfill('0') << std::setw(3) << after_sleep_ms.count() <<
    		'.' << std::setfill('0') << std::setw(3) << after_sleep_in_ns.count() % 1000000 << "\n";

    if((t_run + t_to_active).count() >= frame_time.count()){
    	std::cout << "Time of sleep: already spent"
    			"more time before sleep than needed in the whole frame.\n";
    } else{
    std::cout << "Time of sleep: " << time_of_sleep.count() / 1000000 << "." <<
    		time_of_sleep.count() % 1000000 << "ms, but actually " <<
    		(after_sleep_in_ns.count() - after_run_in_ns.count()) / 1000000 << "." <<
			(after_sleep_in_ns.count() - after_run_in_ns.count()) % 1000000 << "ms\n";
    }

    auto last_after_sleep_in_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    		last_t_after_sleep_.time_since_epoch());
    std::cout << "Time spent in frame: " <<
        		(after_sleep_in_ns.count() - last_after_sleep_in_ns.count()) / 1000000 << "." <<
    			(after_sleep_in_ns.count() - last_after_sleep_in_ns.count()) % 1000000 << "ms\n";

    last_t_after_sleep_ = t_after_sleep_;
  }
}

const char capture_file_path[] =
  "/home/rosdeveloper/ros2_ws/src/ros2_cortex/CaptureWithPlots1.json";
CortexMock mock(capture_file_path);

int Cortex_GetSdkVersion(unsigned char version[4])
{
  version[0] = 0;
  version[1] = 8;
  version[2] = 0;
  version[3] = 0;
  return RC_Okay;
}

int Cortex_SetVerbosityLevel(int i_level)
{
  mock.verbosity_level_ = i_level;
  return RC_Okay;
}

int Cortex_GetVerbosityLevel()
{
  return mock.verbosity_level_;
}

int Cortex_SetMinTimeout(int ms_timeout)
{
  // Timeout has no use here actually,
  // because we don't communicate with the host
  std::string error_msg = "No use of timeout in mock version";
  mock.errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int Cortex_GetMinTimeout()
{
  // Timeout has no use here actually,
  // because we don't communicate with the host
  std::string error_msg = "No use of timeout in mock version";
  mock.errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int Cortex_SetErrorMsgHandlerFunc(
  void (* errorMsgHandlerFunc)(
    int i_log_level,
    char * sz_log_message))
{
  mock.errorMsgHandlerFunc_ = errorMsgHandlerFunc;
  return RC_Okay;
}

int Cortex_SetDataHandlerFunc(void (* dataHandlerFunc)(sFrameOfData * p_frame_of_data))
{
  std::lock_guard<std::mutex> guard(mock.run_cycle_mutex_);
  mock.dataHandlerFunc_ = dataHandlerFunc;
  return RC_Okay;
}

int Cortex_SendDataToClients(sFrameOfData * p_frame_of_data)
{
  // TODO(Gergely Kovacs) send through TCP if client communication is going to be enabled
  std::string error_msg = "No communication with client in mock version";
  mock.errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

void Cortex_SetClientCommunicationEnabled(int b_enabled)
{
  // TODO(Gergely Kovacs) do we enable communication with clients in the mock
  std::string error_msg = "No communication with client in mock version";
  mock.errorMsgInString(VL_Error, error_msg);
}

int Cortex_IsClientCommunicationEnabled()
{
  // TODO(Gergely Kovacs) do we enable communication with clients in the mock
  std::string error_msg = "No communication with client in mock version";
  mock.errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

void Cortex_SetThreadPriorities(
  maThreadPriority listen_for_host,
  maThreadPriority listen_for_data,
  maThreadPriority listen_for_clients)
{
  // TODO(Gergely Kovacs) if communicating with client,
  // should we differentiate priority of listening to data and listening for clients?
  std::string error_msg = "No communication with client neither with host in mock version";
  mock.errorMsgInString(VL_Error, error_msg);
}

int Cortex_ConfigurePortNumbers(
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
  mock.errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int Cortex_Initialize(
  char * sz_talk_to_host_nic_card_address,
  char * sz_host_nic_card_address,
  char * sz_host_multicast_address,
  char * sz_talk_to_clients_nic_card_address,
  char * sz_clients_multicast_address)
{
  std::string error_msg = "No communication with client neither with host in mock version";
  mock.errorMsgInString(VL_Warning, error_msg);
  // TODO(Gergely Kovacs) address storing needs to
  // be implemented if client communication is going to be enabled
  // and complete default initialization, too: sz_host_multicast_address = 225.1.1.1,
  // sz_talk_to_clients_nic_card_address = 127.0.0.1, sz_clients_multicast_address = 225.1.1.2

  mock.readFile();
  mock.current_frame_ind_ = 0;
  mock.run_thread_ = std::thread(&CortexMock::run, &mock);
  return RC_Okay;
}

int Cortex_GetPortNumbers(
  int * talk_to_host_port,
  int * host_port,
  int * host_multicast_port,
  int * talk_to_clients_request_port,
  int * talk_to_clients_multicast_port,
  int * clients_multicast_port)
{
  // TODO(Gergely Kovacs) needs to be implemented if client communication is going to be enabled
  std::string error_msg = "No communication with client neither with host in mock version";
  mock.errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int Cortex_GetAddresses(
  char * sz_talk_to_host_nic_card_address,
  char * sz_host_nic_card_address,
  char * sz_host_multicast_address,
  char * sz_talk_to_clients_nic_card_address,
  char * sz_clients_multicast_address)
{
  // TODO(Gergely Kovacs) needs to be implemented if client communication is going to be enabled
  std::string error_msg = "No communication with client neither with host in mock version";
  mock.errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int Cortex_GetHostInfo(sHostInfo * p_host_info)
{
  // TODO(Gergely Kovacs) needs to be implemented if client communication is going to be enabled
  std::string error_msg = "Found mock version, no communication with host in mock version";
  mock.errorMsgInString(VL_Warning, error_msg);
  return RC_ApiError;
}

int Cortex_Exit()
{
  {
    std::lock_guard<std::mutex> guard(mock.run_cycle_mutex_);
    mock.running_ = false;
  }
  if (mock.run_thread_.joinable()) {mock.run_thread_.join();}
  return RC_Okay;
}

int Cortex_Request(char * sz_command, void ** pp_response, int * pn_bytes)
{
  std::string command(sz_command), command_extra;
  size_t pos = command.find('=');
  if (pos != std::string::npos) {
    command_extra = command.substr(pos);
    command = command.substr(0, pos);
  }
  auto found_it = mock.map_string_to_request.find(command);
  if (found_it == mock.map_string_to_request.end()) {
    std::string error_msg = "Unrecognized request";
    mock.errorMsgInString(VL_Error, error_msg);
    return RC_Unrecognized;
  }
  std::string error_msg = "Mock handles no live mode requests";
  CortexMock::Request req_type = found_it->second;
  std::lock_guard<std::mutex> guard(mock.run_cycle_mutex_);
  switch (req_type) {
    case CortexMock::Request::LiveMode:
      if (!mock.is_in_live_) {
        mock.is_in_live_ = true;
        mock.current_frame_ind_ = 0;
      }
      mock.is_playing_in_live = true;
      break;
    case CortexMock::Request::Pause:
      if (!mock.is_in_live_) {
        mock.is_in_live_ = true;
        mock.current_frame_ind_ = 0;
      }
      mock.is_playing_in_live = false;
      break;
    case CortexMock::Request::SetOutputName:
      break;
    case CortexMock::Request::StartRecording:
      mock.is_recording_in_live = true;
      mock.repeat_num_ = 0;
      mock.actual_repeat_ = 0;
      mock.post_starter_frame_ = mock.current_frame_ind_;
      break;
    case CortexMock::Request::StopRecording:
      mock.is_recording_in_live = false;
      mock.post_end_frame_ = mock.current_frame_ind_;
      break;
    case CortexMock::Request::ResetIDs:
      mock.errorMsgInString(VL_Error, error_msg);
      return RC_ApiError;
    // Mock does deal with post mode requests though
    case CortexMock::Request::PostForward:
      if (mock.is_in_live_) {
        mock.is_in_live_ = false;
        if (mock.is_recording_in_live) {mock.post_end_frame_ = mock.current_frame_ind_;}
        mock.current_frame_ind_ = mock.post_starter_frame_;
      }
      mock.post_play_mode_ = static_cast<int>(CortexMock::PostPlayMode::forwards);
      break;
    case CortexMock::Request::PostBackward:
      if (mock.is_in_live_) {
        mock.is_in_live_ = false;
        if (mock.is_recording_in_live) {mock.post_end_frame_ = mock.current_frame_ind_;}
        mock.current_frame_ind_ = mock.post_starter_frame_;
      }
      mock.post_play_mode_ = static_cast<int>(CortexMock::PostPlayMode::backwards);
      break;
    case CortexMock::Request::PostPause:
      if (mock.is_in_live_) {
        mock.is_in_live_ = false;
        if (mock.is_recording_in_live) {mock.post_end_frame_ = mock.current_frame_ind_;}
        mock.current_frame_ind_ = mock.post_starter_frame_;
      }
      mock.post_play_mode_ = static_cast<int>(CortexMock::PostPlayMode::paused);
      break;
    case CortexMock::Request::PostGetPlayMode:
      *pp_response = &mock.post_play_mode_;
      break;
    case CortexMock::Request::GetContextFrameRate:
      *pp_response = &mock.frame_rate_;
      break;
    case CortexMock::Request::GetContextAnalogSampleRate:
      *pp_response = &mock.analog_sample_rate_;
      break;
    case CortexMock::Request::GetContextAnalogBitDepth:
      *pp_response = &mock.analog_bit_depth_;
      break;
    case CortexMock::Request::GetUpAxis:
      *pp_response = &mock.axis_up_;
      break;
    case CortexMock::Request::GetConversionToMillimeters:
      *pp_response = &mock.conv_rate_to_mm_;
      break;
    case CortexMock::Request::GetFrameOfData:
      if (command_extra.empty()) {
        *pp_response = &mock.current_frame_;
      }
      // TODO(Gergely Kovacs) else return markerset base pos
      break;

    default:
      error_msg = "Unrecognized request";
      mock.errorMsgInString(VL_Error, error_msg);
      return RC_Unrecognized;
  }

  return RC_Okay;
}

sSkyReturn * Cortex_SkyCommand(char * sz_command, int ms_timeout)
{
  // TODO(Gergely Kovacs) implement this function
  std::string error_msg = "Function not implemented yet";
  mock.errorMsgInString(VL_Error, error_msg);
  return nullptr;
}

sBodyDefs * Cortex_GetBodyDefs()
{
  return &mock.body_defs_;
}

int Cortex_FreeBodyDefs(sBodyDefs * p_body_defs)
{
  // Free bodydefs
  if (p_body_defs == nullptr) {return RC_Okay;}
  int n_body_defs = p_body_defs->nBodyDefs;
  for (int i = 0; i < n_body_defs; ++i) {
    mock.freeBodyDef(p_body_defs->BodyDefs[i], p_body_defs->nAnalogChannels);
  }

  // Free analog data
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

sFrameOfData * Cortex_GetCurrentFrame()
{
  std::lock_guard<std::mutex> guard(mock.run_cycle_mutex_);
  mock.extractFrame(mock.current_frame_, mock.current_frame_ind_);
  return &mock.current_frame_;
}

int Cortex_CopyFrame(const sFrameOfData * p_src, sFrameOfData * p_dst)
{
  p_dst->iFrame = p_src->iFrame;
  p_dst->fDelay = p_src->fDelay;

  // Copy body datas
  p_dst->nBodies = p_src->nBodies;
  int n_bodies = p_dst->nBodies;
  for (int i = 0; i < n_bodies; i++) {
    mock.copyBodyData(p_src->BodyData[i], p_dst->BodyData[i]);
  }

  // Copy unidentified markers
  p_dst->nUnidentifiedMarkers = p_src->nUnidentifiedMarkers;
  int n_ui_markers = p_dst->nUnidentifiedMarkers;
  if (n_ui_markers > 0) {
    p_dst->UnidentifiedMarkers = new tMarkerData[n_ui_markers];
    memcpy(
      p_dst->UnidentifiedMarkers, p_src->UnidentifiedMarkers,
      n_ui_markers * sizeof(tMarkerData));
  }

  mock.copyAnalogData(p_src->AnalogData, p_dst->AnalogData);

  // Copy recording data
  p_dst->RecordingStatus.bRecording = p_src->RecordingStatus.bRecording;
  p_dst->RecordingStatus.iFirstFrame = p_src->RecordingStatus.iFirstFrame;
  p_dst->RecordingStatus.iLastFrame = p_src->RecordingStatus.iLastFrame;
  memcpy(
    p_dst->RecordingStatus.szFilename, p_src->RecordingStatus.szFilename,
    strlen(p_src->RecordingStatus.szFilename) + 1);

  // Copy time data
  p_dst->TimeCode.iFrames = p_src->TimeCode.iFrames;
  p_dst->TimeCode.iHours = p_src->TimeCode.iHours;
  p_dst->TimeCode.iMinutes = p_src->TimeCode.iMinutes;
  p_dst->TimeCode.iSeconds = p_src->TimeCode.iSeconds;
  p_dst->TimeCode.iStandard = p_src->TimeCode.iStandard;

  return RC_Okay;
}

int Cortex_FreeFrame(sFrameOfData * p_frame)
{
  // Free body datas
  if (p_frame == nullptr) {return RC_Okay;}
  int n_bodies = p_frame->nBodies;
  if (n_bodies > 0) {
    for (int i_body = 0; i_body < n_bodies; ++i_body) {
      mock.freeBodyData(p_frame->BodyData[i_body]);
    }
  }

  // Free unidentified markers and analog data
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

int Cortex_SendHtr(sHierarchy * p_hierarchy, tSegmentData * p_frame)
{
  // TODO(Gergely Kovacs) implement this function
  std::string error_msg = "Function not implemented yet";
  mock.errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

int Cortex_SetMetered(bool b_active, float f_fixed_latency)
{
  // TODO(Gergely Kovacs) if we enable communication with clients in the mock
  std::string error_msg = "No communication with client in mock version";
  mock.errorMsgInString(VL_Error, error_msg);
  return RC_ApiError;
}

void Cortex_ConstructRotationMatrix(
  double angles[3], int i_rotation_order,
  double matrix[3][3])
{
  // TODO(Gergely Kovacs) implement this function
  std::string error_msg = "Function not implemented yet";
  mock.errorMsgInString(VL_Error, error_msg);
}

void Cortex_ExtractEulerAngles(
  double matrix[3][3], int i_rotation_order,
  double angles[3])
{
  // TODO(Gergely Kovacs) implement this function
  std::string error_msg = "Function not implemented yet";
  mock.errorMsgInString(VL_Error, error_msg);
}

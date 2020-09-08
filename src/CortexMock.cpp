#include <chrono>
#include <thread>

#include "CortexMock.hpp"
#include "rapidjson/filereadstream.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

CortexMock::CortexMock(const std::string& capture_file_name):capture_file_name_(capture_file_name) {
	initReadFile();
}

// CortexMock::CortexMock(const CortexMock& src){
// 	src.getCaptureFilename(capture_file_name_);
// 	// TODO copy other variables??
// 	initReadFile();
// }

void CortexMock::initReadFile(){
	FILE* fp = fopen(capture_file_name_.data(), "r");
	char read_buffer[65536];
	rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer));

	document_.ParseStream(is);
	fclose(fp);
	n_frames_ = document_["framesArray"].Size();
	// TODO captur bodydefs too for the json files
	// extractBodyDefs(body_defs_, document_["bodyDefs"]);
}

CortexMock::~CortexMock(){
	freeFrame(&current_frame_);
}

void CortexMock::getCaptureFilename(std::string& dest) const{
	dest = capture_file_name_;
}

int CortexMock::getSdkVersion(unsigned char Version[4]){
    Version[0] = 0;
    Version[1] = 8;
    Version[2] = 0;
    Version[3] = 0;
    return RC_Okay;
}

int CortexMock::setVerbosityLevel(int iLevel){
    // TODO Logging?
	verbosity_level_ = iLevel;
    return RC_Okay;
}

int CortexMock::getVerbosityLevel(){
    // TODO Logging?
    return verbosity_level_;
}

int CortexMock::setMinTimeout(int msTimeout){
    // Does using timeout make sense?
    min_time_out_ = msTimeout;
    return RC_Okay;
}

int CortexMock::getMinTimeout(){
    return min_time_out_;
}

int CortexMock::setErrorMsgHandlerFunc(void (*errorMsgHandlerFunc)(int iLogLevel, char* szLogMessage)){
	errorMsgHandlerFunc_ = errorMsgHandlerFunc;
	return RC_Okay;
}

int CortexMock::setDataHandlerFunc(void (*dataHandlerFunc)(sFrameOfData* pFrameOfData)){
	dataHandlerFunc_ = dataHandlerFunc;
	return RC_Okay;
}

int CortexMock::sendDataToClients(sFrameOfData* pFrameOfData){
	// TODO send through TCP? or what way?
	return RC_GeneralError;
}

void CortexMock::setClientCommunicationEnabled(int bEnabled){
	client_comm_enabled_ = bEnabled;
}

int CortexMock::isClientCommunicationEnabled(){
	return client_comm_enabled_;
}

void CortexMock::setThreadPriorities(maThreadPriority ListenForHost, maThreadPriority ListenForData, maThreadPriority ListenForClients){
	// TODO
}

int CortexMock::configurePortNumbers(int TalkToHostPort,
										int HostPort, 
										int HostMulticastPort, 
										int TalkToClientsRequestPort,
										int TalkToClientsMulticastPort,
										int ClientsMulticastPort){
    
    talk_to_host_port_ = TalkToHostPort;
    host_port_ = HostPort;
    host_multicast_port_ = HostMulticastPort;
    talk_to_clients_request_port_ = TalkToClientsRequestPort;
    talk_to_clients_multicast_port_ = TalkToClientsMulticastPort;
    clients_multicast_port_ = ClientsMulticastPort;
    
    // TODO actually change ports

    return RC_Okay;
}

int CortexMock::initialize(	char* szTalkToHostNicCardAddress,
							char* szHostNicCardAddress,
							char* szHostMulticastAddress,
							char* szTalkToClientsNicCardAddress,
							char* szClientsMulticastAddress){
	inet_aton(szHostNicCardAddress, &host_machine_address_);
	inet_aton(szHostMulticastAddress, &host_multicast_address_);
	inet_aton(szTalkToHostNicCardAddress, &talk_to_host_address_);
	inet_aton(szTalkToClientsNicCardAddress, &talk_to_client_address_);
	inet_aton(szClientsMulticastAddress, &client_multicast_address_);

	pthread_create(&run_thread_, nullptr, &CortexMock::run_helper, this);
	return RC_Okay;
}

int CortexMock::getPortNumbers(	int *TalkToHostPort,
								int *HostPort, 
								int *HostMulticastPort, 
								int *TalkToClientsRequestPort,
								int *TalkToClientsMulticastPort,
								int *ClientsMulticastPort){
    *TalkToHostPort = talk_to_host_port_;
    *HostPort = host_port_;
    *HostMulticastPort = host_multicast_port_;
    *TalkToClientsRequestPort = talk_to_clients_request_port_;
    *TalkToClientsMulticastPort = talk_to_clients_multicast_port_;
    *ClientsMulticastPort = clients_multicast_port_;
    return RC_Okay;
}

int CortexMock::getAddresses(char* szTalkToHostNicCardAddress,
							char* szHostNicCardAddress,
							char* szHostMulticastAddress,
							char* szTalkToClientsNicCardAddress,
							char* szClientsMulticastAddress){
    szTalkToHostNicCardAddress = inet_ntoa(talk_to_host_address_);
    szHostNicCardAddress = inet_ntoa(host_machine_address_);
    szHostMulticastAddress = inet_ntoa(host_multicast_address_);
    szTalkToClientsNicCardAddress = inet_ntoa(talk_to_client_address_);
    szClientsMulticastAddress = inet_ntoa(client_multicast_address_);

	return RC_Okay;
}

int CortexMock::getHostInfo(sHostInfo *pHostInfo){
    pHostInfo->bFoundHost = true;
    pHostInfo->LatestConfirmationTime = current_framenum_;
    strcpy(pHostInfo->szHostMachineName, "CortexHost");
    inet_aton((char*)pHostInfo->HostMachineAddress, &host_machine_address_);
    getSdkVersion(pHostInfo->HostProgramVersion);
    strcpy(pHostInfo->szHostProgramName, "CortexMock");

    //TODO Error handling
    return RC_Okay;
}

int CortexMock::exit(){
	client_comm_enabled_ = false;
	play_mode_ = static_cast<int>(PlayMode::paused);
	return RC_Okay;
}

int CortexMock::request(char* szCommand, void** ppResponse, int *pnBytes){
	std::string command(szCommand), command_extra;
	int pos = command.find('=');
	if(pos != std::string::npos){
		command_extra = command.substr(pos);
		command = command.substr(0, pos);
	}
	auto found_it = map_string_to_request.find(command);
	if(found_it == map_string_to_request.end()) return RC_Unrecognized;
	Request req_type = found_it->second;
	switch (req_type)
	{
	// Mock doesn't and can't deal with live mode requests
	// TODO should it?
	case Request::LiveMode:
		return RC_GeneralError;
	case Request::Pause:
		return RC_GeneralError;
	case Request::SetOutputName:
		return RC_GeneralError;
	case Request::StartRecording:
		return RC_GeneralError;
	case Request::StopRecording:
		return RC_GeneralError;
	case Request::ResetIDs:
		return RC_GeneralError;
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
		*ppResponse = &play_mode_;
		break;
	case Request::GetContextFrameRate:
		*ppResponse = &frame_rate_;
		break;
	case Request::GetContextAnalogSampleRate:
		*ppResponse = &analog_sample_rate_;
		break;
	case Request::GetContextAnalogBitDepth:
		*ppResponse = &analog_bit_depth_;
		break;
	case Request::GetUpAxis:
		*ppResponse = &axis_up_;
		break;
	case Request::GetConversionToMillimeters:
		*ppResponse = &conv_rate_to_mm_;
		break;
	case Request::GetFrameOfData:
		if(command_extra.empty()) *ppResponse = &current_frame_;
		// TODO else return markerset base pos
		break;
	
	default:
		return RC_Unrecognized;
	}

	return RC_Okay;
}

sSkyReturn CortexMock::*skyCommand(char *szCommand, int msTimeout){
	// TODO
}

sBodyDefs* CortexMock::getBodyDefs(){
	return &body_defs_;
}

int CortexMock::freeBodyDefs(sBodyDefs* pBodyDefs){
	int n_body_defs = pBodyDefs->nBodyDefs;
	for (int i = 0; i < n_body_defs; ++i)
	{
		delete [] pBodyDefs->BodyDefs[i].szName;

		int n_markers = pBodyDefs->nAnalogChannels;
		for (int i_marker_name = 0; i_marker_name < n_markers; ++i_marker_name) {
			if(pBodyDefs->BodyDefs[i].szMarkerNames[i_marker_name] != nullptr) delete [] pBodyDefs->BodyDefs[i].szMarkerNames[i_marker_name];
		}
		if(n_markers > 0) delete [] pBodyDefs->BodyDefs[i].szMarkerNames;

		int n_segments = pBodyDefs->BodyDefs[i].Hierarchy.nSegments;
		for (int i_segment_name = 0; i_segment_name < n_segments; ++i_segment_name) {
			if(pBodyDefs->BodyDefs[i].Hierarchy.szSegmentNames[i_segment_name] != nullptr)
				delete [] pBodyDefs->BodyDefs[i].Hierarchy.szSegmentNames[i_segment_name];
		}
		if(n_segments > 0){
			delete [] pBodyDefs->BodyDefs[i].Hierarchy.szSegmentNames;
			delete [] pBodyDefs->BodyDefs[i].Hierarchy.iParents;
		}

		int n_dofs = pBodyDefs->nAnalogChannels;
		for (int i_dof_name = 0; i_dof_name < n_dofs; ++i_dof_name) {
			if(pBodyDefs->BodyDefs[i].szDofNames[i_dof_name] != nullptr) delete [] pBodyDefs->BodyDefs[i].szDofNames[i_dof_name];
		}
		if(n_markers > 0) delete [] pBodyDefs->BodyDefs[i].szDofNames;
	}
	

	int n_analogch = pBodyDefs->nAnalogChannels;
	for (int i_ach_name = 0; i_ach_name < n_analogch; ++i_ach_name) {
		if(pBodyDefs->szAnalogChannelNames[i_ach_name] != nullptr) delete [] pBodyDefs->szAnalogChannelNames[i_ach_name];
	}
	if(n_analogch > 0) delete [] pBodyDefs->szAnalogChannelNames;
	delete[] pBodyDefs->AnalogLoVoltage;
	delete[] pBodyDefs->AnalogHiVoltage;
	// TODO should i delete pBodyDefs->AllocatedSpace??
	return RC_Okay;
}

sFrameOfData* CortexMock::getCurrentFrame(){
	extractFrame(current_frame_, current_framenum_);
	return &current_frame_;
}

int CortexMock::copyFrame(const sFrameOfData* pSrc, sFrameOfData* pDst){
	pDst->iFrame = pSrc->iFrame;
	pDst->fDelay = pSrc->fDelay;
	int n_bodies = pDst->nBodies = pSrc->nBodies;
	for (int i = 0; i < n_bodies; i++)
	{
		strcpy(pDst->BodyData[i].szName, pSrc->BodyData[i].szName);

		int n_markers = pDst->BodyData[i].nMarkers = pSrc->BodyData[i].nMarkers;
		if(n_markers > 0){
			pDst->BodyData[i].Markers = new tMarkerData[n_markers];
			memcpy(pDst->BodyData[i].Markers, pSrc->BodyData[i].Markers, n_markers * sizeof(tMarkerData));
		}
		pDst->BodyData[i].fAvgMarkerResidual = pSrc->BodyData[i].fAvgMarkerResidual;

		int n_segments = pDst->BodyData[i].nSegments = pSrc->BodyData[i].nSegments;
		if(n_segments > 0){
			pDst->BodyData[i].Segments = new tSegmentData[n_segments];
			memcpy(pDst->BodyData[i].Segments, pSrc->BodyData[i].Segments, n_segments * sizeof(tSegmentData));
		}

		int n_dofs = pDst->BodyData[i].nDofs = pSrc->BodyData[i].nDofs;
		if(n_dofs > 0){
			pDst->BodyData[i].Dofs = new tDofData[n_dofs];
			memcpy(pDst->BodyData[i].Dofs, pSrc->BodyData[i].Dofs, n_dofs * sizeof(tDofData));
		}
		pDst->BodyData[i].fAvgDofResidual = pSrc->BodyData[i].fAvgDofResidual;
		pDst->BodyData[i].nIterations = pSrc->BodyData[i].nIterations;

		pDst->BodyData[i].ZoomEncoderValue = pSrc->BodyData[i].ZoomEncoderValue;
		pDst->BodyData[i].FocusEncoderValue = pSrc->BodyData[i].FocusEncoderValue;
		pDst->BodyData[i].IrisEncoderValue = pSrc->BodyData[i].IrisEncoderValue;
		memcpy(pDst->BodyData[i].CamTrackParams, pSrc->BodyData[i].CamTrackParams, sizeof(tCamTrackParameters));

		int n_events = pDst->BodyData[i].nEvents = pSrc->BodyData[i].nEvents;
		char** src_event_str_ptr = pSrc->BodyData[i].Events;
		pDst->BodyData[i].Events = new char*[n_events];
		char** dst_event_str_ptr = pDst->BodyData[i].Events;
		// TODO test this and other functionalities too which aren't tested by capture files
		for (int i = 0; i < n_events; ++i, ++src_event_str_ptr, ++dst_event_str_ptr)
		{
			std::string event_str(*src_event_str_ptr);
            *dst_event_str_ptr = new char[event_str.length()+1];
			strcpy(*dst_event_str_ptr, event_str.data());
		}
	}

	int n_ui_markers = pDst->nUnidentifiedMarkers = pSrc->nUnidentifiedMarkers;
    if(n_ui_markers > 0){
		pDst->UnidentifiedMarkers = new tMarkerData[n_ui_markers];
		memcpy(pDst->UnidentifiedMarkers, pSrc->UnidentifiedMarkers, n_ui_markers * sizeof(tMarkerData));
	}

	pDst->AnalogData.nAnalogChannels = pSrc->AnalogData.nAnalogChannels;
	pDst->AnalogData.nAnalogSamples = pSrc->AnalogData.nAnalogSamples;
	int n_analogs = pDst->AnalogData.nAnalogChannels * pDst->AnalogData.nAnalogSamples;
	if(n_analogs > 0){
		pDst->AnalogData.AnalogSamples = new short[n_analogs];
		memcpy(pDst->AnalogData.AnalogSamples, pSrc->AnalogData.AnalogSamples, n_analogs * sizeof(short));
	}

	pDst->AnalogData.nForcePlates = pSrc->AnalogData.nForcePlates;
	pDst->AnalogData.nForceSamples = pSrc->AnalogData.nForceSamples;
	int n_forces = pDst->AnalogData.nForcePlates * pDst->AnalogData.nForceSamples;
	if(n_forces > 0){
		pDst->AnalogData.Forces = new tForceData[n_forces];
		memcpy(pDst->AnalogData.Forces, pSrc->AnalogData.Forces, n_forces * sizeof(tForceData));
	}
	

	pDst->AnalogData.nAngleEncoders = pSrc->AnalogData.nAngleEncoders;
	pDst->AnalogData.nAngleEncoderSamples = pSrc->AnalogData.nAngleEncoderSamples;
	int n_all_ae_samples = pDst->AnalogData.nAngleEncoders * pDst->AnalogData.nAngleEncoderSamples;
	if(n_analogs > 0){
		pDst->AnalogData.AngleEncoderSamples = new double[n_all_ae_samples];
		memcpy(pDst->AnalogData.AngleEncoderSamples, pSrc->AnalogData.AngleEncoderSamples, n_all_ae_samples * sizeof(double));
	}

    pDst->RecordingStatus.bRecording = pSrc->RecordingStatus.bRecording;
	pDst->RecordingStatus.iFirstFrame = pSrc->RecordingStatus.iFirstFrame;
	pDst->RecordingStatus.iLastFrame = pSrc->RecordingStatus.iLastFrame;
	strcpy(pDst->RecordingStatus.szFilename, pSrc->RecordingStatus.szFilename);

	pDst->TimeCode.iFrames = pSrc->TimeCode.iFrames;
	pDst->TimeCode.iHours = pSrc->TimeCode.iHours;
	pDst->TimeCode.iMinutes = pSrc->TimeCode.iMinutes;
	pDst->TimeCode.iSeconds = pSrc->TimeCode.iSeconds;
	pDst->TimeCode.iStandard = pSrc->TimeCode.iStandard;

	return RC_GeneralError;
}

int CortexMock::freeFrame(sFrameOfData* pFrame){
	int n_bodies = pFrame->nBodies;
	if(n_bodies > 0){
		for (int i_body = 0; i_body < n_bodies; ++i_body) {
			sBodyData& i_body_data = pFrame->BodyData[i_body];
			if(i_body_data.nMarkers > 0) delete [] i_body_data.Markers;
			if(i_body_data.nSegments > 0) delete [] i_body_data.Segments;
			if(i_body_data.nDofs > 0) delete [] i_body_data.Dofs;
			int n_events = i_body_data.nEvents;
			for (int i_event = 0; i_event < n_events; ++i_event) {
				if(i_body_data.Events[i_event] != nullptr) delete [] i_body_data.Events[i_event];
			}
			if(i_body_data.nEvents > 0) delete [] i_body_data.Events;
		}
	}

	if(pFrame->nUnidentifiedMarkers > 0) delete [] pFrame->UnidentifiedMarkers;
	if(pFrame->AnalogData.nAnalogSamples > 0 && pFrame->AnalogData.nAnalogChannels > 0) delete [] pFrame->AnalogData.AnalogSamples;
	if(pFrame->AnalogData.nForceSamples > 0 && pFrame->AnalogData.nForcePlates > 0) delete [] pFrame->AnalogData.Forces;
	if(pFrame->AnalogData.nAngleEncoderSamples > 0 && pFrame->AnalogData.nAngleEncoders > 0) delete [] pFrame->AnalogData.AngleEncoderSamples;
	return RC_Okay;
}

int CortexMock::sendHtr(sHierarchy *pHierarchy, tSegmentData *pFrame){
	// TODO
	return RC_GeneralError;
}

int CortexMock::setMetered(bool bActive, float fFixedLatency){
	// TODO
	return RC_GeneralError;
}

void CortexMock::constructRotationMatrix(double angles[3], int iRotationOrder, double matrix[3][3]){
	// TODO
}

void CortexMock::extractEulerAngles(double matrix[3][3],int iRotationOrder, double angles[3]){
	// TODO
}

void CortexMock::extractBodyDefs(sBodyDefs& body_defs, const rapidjson::Value& body_defs_json){
    int n_body_defs = body_defs.nBodyDefs = body_defs_json["nBodyDefs"].GetInt();
    rapidjson::Value body_def_array(rapidjson::kArrayType);
	for (int i = 0; i < n_body_defs; i++)
	{
		extractBodyDef(body_defs.BodyDefs[i], body_def_array[i]);
	}

    int n_analog_channels = body_defs.nAnalogChannels = body_defs_json["nAnalogChannels"].GetInt();
	body_defs.szAnalogChannelNames = new char*[n_analog_channels];
	char** dst_analogch_names_ptr = body_defs.szAnalogChannelNames;
	for (int i = 0; i < n_analog_channels; ++i, ++dst_analogch_names_ptr)
	{
		std::string analogch_name = body_defs_json["analogChannelNames"][i].GetString();
        *dst_analogch_names_ptr = new char[analogch_name.length()+1];
		strcpy(*dst_analogch_names_ptr, analogch_name.data());
	}

	body_defs.nForcePlates = body_defs_json["nForcePlates"].GetInt();
	body_defs.AnalogBitDepth = body_defs_json["analogBitDepth"].GetInt();

	body_defs.AnalogLoVoltage = new float[n_analog_channels];
	body_defs.AnalogHiVoltage = new float[n_analog_channels];
	for (int i = 0; i < n_analog_channels; i++)
	{
		body_defs.AnalogLoVoltage[i] = body_defs_json["analogLoVoltage"][i].GetFloat();
		body_defs.AnalogHiVoltage[i] = body_defs_json["analogHiVoltage"][i].GetFloat();
	}
}

void CortexMock::extractBodyDef(sBodyDef& body_def, const rapidjson::Value& body_def_json){
	std::string name(body_def_json["name"].GetString());
	body_def.szName = new char[name.length()+1];
	strcpy(body_def.szName, name.data());

	int n_markers = body_def.nMarkers = body_def_json["nMarkers"].GetInt();
	body_def.szMarkerNames = new char*[n_markers];
	char** dst_marker_names_ptr = body_def.szMarkerNames;
	for (int i = 0; i < n_markers; ++i, ++dst_marker_names_ptr)
	{
		std::string marker_name = body_def_json["markerNames"][i].GetString();
        *dst_marker_names_ptr = new char[marker_name.length()+1];
		strcpy(*dst_marker_names_ptr, marker_name.data());
	}

	int n_segments = body_def.Hierarchy.nSegments = body_def_json["hierarchy"]["nSegments"].GetInt();
	body_def.Hierarchy.szSegmentNames = new char*[n_segments];
	char** dst_segment_names_ptr = body_def.Hierarchy.szSegmentNames;
	body_def.Hierarchy.iParents = new int[n_segments];
	for (int i = 0; i < n_segments; ++i, ++dst_segment_names_ptr)
	{
		std::string segment_name = body_def_json["hierarchy"]["segmentNames"][i].GetString();
        *dst_segment_names_ptr = new char[segment_name.length()+1];
		strcpy(*dst_segment_names_ptr, segment_name.data());

		body_def.Hierarchy.iParents[i] = body_def_json["hierarchy"]["parents"][i].GetInt();
	}

	int n_dofs = body_def.nDofs = body_def_json["nDofs"].GetInt();
	body_def.szDofNames = new char*[n_dofs];
	char** dst_dof_names_ptr = body_def.szDofNames;
	for (int i = 0; i < n_dofs; ++i, ++dst_dof_names_ptr)
	{
		std::string dof_name = body_def_json["dofNames"][i].GetString();
        *dst_dof_names_ptr = new char[dof_name.length()+1];
		strcpy(*dst_dof_names_ptr, dof_name.data());
	}
}

void CortexMock::extractBodies(sFrameOfData& fod, const rapidjson::Value& parent_value){
	int n_bodies = fod.nBodies; 
	for (int i = 0; i < n_bodies; ++i) {
		const rapidjson::Value& i_body_json = parent_value["bodies"][i];
		sBodyData& i_body_data = fod.BodyData[i];
		strcpy(i_body_data.szName,i_body_json["name"].GetString());
		i_body_data.nMarkers = i_body_json["nMarkers"].GetInt();
		if(i_body_data.nMarkers > 0){
			i_body_data.Markers = new tMarkerData[i_body_data.nMarkers];
			extractMarkers(i_body_data.Markers, i_body_data.nMarkers, i_body_json["markers"]);
		}
		i_body_data.fAvgMarkerResidual = i_body_json["fAvgMarkerResidual"].GetFloat();
		i_body_data.nSegments = i_body_json["nSegments"].GetInt();
		if(i_body_data.nSegments > 0){
			i_body_data.Segments = new tSegmentData[i_body_data.nSegments];
			extractSegments(i_body_data.Segments, i_body_data.nSegments, i_body_json["segments"]);
		}
		i_body_data.nDofs = i_body_json["nDofs"].GetInt();

		int n_dofs = i_body_data.nDofs;
		if(n_dofs > 0){
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

		const rapidjson::Value& cam_track_params = i_body_json["camTrackParams"];
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
		if(n_events > 0){
			i_body_data.Events = new char*[n_events];
			for (int i_event = 0; i_event < n_events; ++i_event) {
				i_body_data.Events[i_event] = new char[i_body_json["events"][i].GetStringLength()];
				strcpy(i_body_data.Events[i_event], i_body_json["events"][i].GetString());
			}
		}
	}
}

void CortexMock::extractMarkers(tMarkerData* markers, int n_markers, const rapidjson::Value& parent_value){
	for (int i_marker = 0; i_marker < n_markers; ++i_marker) {
		markers[i_marker][0] = parent_value[i_marker]["x"].GetFloat();
		markers[i_marker][1] = parent_value[i_marker]["y"].GetFloat();
		markers[i_marker][2] = parent_value[i_marker]["z"].GetFloat();
	}
}

void CortexMock::extractAnalogData(sAnalogData& adata, const rapidjson::Value& parent_value){
	int n_channels = adata.nAnalogChannels = parent_value["nAnalogChannels"].GetInt();
	int n_samples = adata.nAnalogSamples = parent_value["nAnalogSamples"].GetInt();
	int index = 0;
	if(n_channels > 0 || n_samples > 0){
		adata.AnalogSamples = new short[n_samples*n_channels];
		for (int i_sample=0 ; i_sample<n_samples ; i_sample++)
		{
			for (int i_channel=0 ; i_channel<n_channels ; i_channel++)
			{
				index = i_sample*n_samples+i_channel;
				adata.AnalogSamples[index] = parent_value["analogSamples"][index]["value"].GetInt();
			}
		}
	}

	int n_force_plates = adata.nForcePlates = parent_value["nForcePlates"].GetInt();
	int n_force_samples = adata.nForceSamples = parent_value["nForceSamples"].GetInt();
	if(n_force_plates > 0 || n_force_samples > 0){
		adata.Forces = new tForceData[n_force_samples*n_force_plates];
		for (int i_sample=0; i_sample<n_force_samples; i_sample++)
		{
			for (int i_plate=0; i_plate<n_force_plates; i_plate++)
			{
				index = i_sample*n_samples+i_plate;
				const rapidjson::Value& force_json = parent_value["forces"][index];
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

	int n_angle_encoders = adata.nAngleEncoders = parent_value["nAngleEncoders"].GetInt();
	int n_angle_encoder_samples = adata.nAngleEncoderSamples = parent_value["nAngleEncoderSamples"].GetInt();
	if(n_angle_encoders > 0 || n_angle_encoder_samples > 0){
		adata.AngleEncoderSamples = new double[n_angle_encoders*n_angle_encoder_samples];
		for (int i_sample=0 ; i_sample<n_angle_encoder_samples ; i_sample++)
		{
			for (int i_enc=0 ; i_enc<n_angle_encoders ; i_enc++)
			{
				index = i_sample*n_angle_encoder_samples+i_enc;
				adata.AngleEncoderSamples[index] = parent_value["angleEncoderSamples"][index]["value"].GetDouble();
			}
		}
	}
}

void CortexMock::extractSegments(tSegmentData* segments, int n_segments, const rapidjson::Value& parent_value){
	for (int i_segment = 0; i_segment < n_segments; ++i_segment) {
		segments[i_segment][0] = parent_value[i_segment]["x"].GetDouble();
		segments[i_segment][1] = parent_value[i_segment]["y"].GetDouble();
		segments[i_segment][2] = parent_value[i_segment]["z"].GetDouble();
		segments[i_segment][3] = parent_value[i_segment]["aX"].GetDouble();
		segments[i_segment][4] = parent_value[i_segment]["aY"].GetDouble();
		segments[i_segment][5] = parent_value[i_segment]["aZ"].GetDouble();
		segments[i_segment][6] = parent_value[i_segment]["length"].GetDouble();
	}
}

void CortexMock::extractFrame(sFrameOfData& fod, int iFrame){
	freeFrame(&fod);
	const rapidjson::Value& frame = document_["framesArray"][iFrame];
	fod.iFrame = frame["frame"].GetInt();
	fod.fDelay = frame["frameDelay"].GetFloat();
	fod.nBodies = frame["nBodies"].GetInt();

	if(fod.nBodies > 0) extractBodies(fod, frame);

	fod.nUnidentifiedMarkers = frame["nUnidentifiedMarkers"].GetInt();

	if(fod.nUnidentifiedMarkers > 0){
		fod.UnidentifiedMarkers = new tMarkerData[fod.nUnidentifiedMarkers];
		extractMarkers(fod.UnidentifiedMarkers, fod.nUnidentifiedMarkers, frame["unidentifiedMarkers"]);
	}

	extractAnalogData(fod.AnalogData, frame["analogData"]);

	const rapidjson::Value& rc_status = frame["recordingStatus"];
	fod.RecordingStatus.bRecording = rc_status["recording"].GetInt();
	fod.RecordingStatus.iFirstFrame = rc_status["firstFrame"].GetInt();
	fod.RecordingStatus.iLastFrame = rc_status["lastFrame"].GetInt();
	strcpy(fod.RecordingStatus.szFilename, rc_status["captureFileName"].GetString());

	const rapidjson::Value& tc_value = frame["timeCode"];
	fod.TimeCode.iHours = tc_value["hours"].GetInt();
	fod.TimeCode.iMinutes = tc_value["minutes"].GetInt();
	fod.TimeCode.iSeconds = tc_value["seconds"].GetInt();
	fod.TimeCode.iFrames = tc_value["frames"].GetInt();

	// Correct this by changing generation to simple int instead of converting twice
	std::string standard = tc_value["standard"].GetString();
	if(standard == "SMPTE"){
		fod.TimeCode.iStandard = 1;
	} else if(standard == "FILM"){
		fod.TimeCode.iStandard = 2;
	} else if(standard == "EBU"){
		fod.TimeCode.iStandard = 3;
	} else if(standard == "SYSTEMCLOCK"){
		fod.TimeCode.iStandard = 4;
	}
}

void* CortexMock::run_helper(void* cortex_mock){
    static_cast<CortexMock*>(cortex_mock)->run();
	return nullptr;
}

void CortexMock::run(){
	while(client_comm_enabled_) {
		switch (static_cast<PlayMode>(play_mode_))
		{
		case PlayMode::forwards:
			extractFrame(current_frame_, current_framenum_);
			dataHandlerFunc_(&current_frame_);
			current_framenum_ = current_framenum_ < n_frames_-1 ? current_framenum_+1 : 0;
			break;
		case PlayMode::backwards:
			extractFrame(current_frame_, current_framenum_);
			dataHandlerFunc_(&current_frame_);
			current_framenum_ = current_framenum_ > 0 ? current_framenum_-1 : n_frames_-1;
			break;

		default:
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(1000.0/frame_rate_)));
	}
}

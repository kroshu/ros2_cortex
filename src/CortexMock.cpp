#include <string.h>


#include "CortexMock.hpp"
#include "rapidjson/filereadstream.h"

CortexMock::CortexMock(const std::string capture_file_name):capture_file_name_(capture_file_name) {
	FILE* fp = fopen((const char*)capture_file_name_, "r");
	char read_buffer[65536];
	rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer));

	document.ParseStream(is);
	fclose(fp);
	n_frames = document["framesArray"].Size();
}

CortexMock::~CortexMock(){
	freeFrame(&current_frame_);
}

int CortexMock::getSdkVersion(unsigned char Version[4]){
    Version[0] = 0;
    Version[1] = 8;
    Version[2] = 0;
    Version[3] = 0;
    return RC_Okay;
}

int CortexMock::setVerbosityLevel(int iLevel){
    //TODO Logging?
    return RC_GeneralError;
}

int CortexMock::getVerbosityLevel(){
    //TODO Logging?
    return VL_Warning;
}

int CortexMock::setMinTimeout(int msTimeout){
    //Does using timeout make sense?
    min_time_out_ = msTimeout;
    return RC_Okay;
}

int CortexMock::getMinTimeout(){
    return min_time_out_;
}

int CortexMock::setErrorMsgHandlerFunc(void (*MyFunction)(int iLogLevel, char* szLogMessage)){
	// TODO
	return RC_GeneralError;
}

int CortexMock::setDataHandlerFunc(void (*MyFunction)(sFrameOfData* pFrameOfData)){
	// TODO
	return RC_GeneralError;
}

int CortexMock::sendDataToClients(sFrameOfData* pFrameOfData){
	// TODO
	return RC_GeneralError;
}

void CortexMock::setClientCommunicationEnabled(int bEnabled){
	// TODO
}

int CortexMock::isClientCommunicationEnabled(){
	// TODO
	return RC_GeneralError;
}

void CortexMock::setThreadPriorities(maThreadPriority ListenForHost, maThreadPriority ListenForData, maThreadPriority ListenForClients){
	// TODO
}

int CortexMock::configurePortNumbers(int TalkToHostPort,
										int HostPort, 
										int HostMulticastPort, 
										int TalkToClientsRequestPort = 0,
										int TalkToClientsMulticastPort = 0,
										int ClientsMulticastPort = -1){
    
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
							char* szHostMulticastAddress = (char*)"225.1.1.1",
							char* szTalkToClientsNicCardAddress = 0,
							char* szClientsMulticastAddress = (char*)"225.1.1.2"){
	inet_aton(szHostNicCardAddress, &host_machine_address_);
	inet_aton(szHostMulticastAddress, &host_multicast_address_);
	inet_aton(szTalkToHostNicCardAddress, &talk_to_host_address_);
	inet_aton(szTalkToClientsNicCardAddress, &talk_to_client_address_);
	inet_aton(szClientsMulticastAddress, &host_machine_address_);

	//establish connection here
	try {
	    tcp_connection_ = std::make_unique<kuka_sunrise::TCPConnection>(
	      talk_to_client_address_,
	      talk_to_clients_request_port_,
	      [this](sFrameOfData* pFrameOfData) {this->dataHandlerFunc(pFrameOfData);},
	      [this](const char * server_addr,
	      int server_port) {this->connectionLostCallback(&talk_to_client_address_, talk_to_clients_request_port_);});
	  } catch (...) {
	    tcp_connection_.reset();
	  }

	run();
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
    pHostInfo->LatestConfirmationTime = current_frame_;
    strcpy(pHostInfo->szHostMachineName, "CortexHost");
    inet_aton((char*)pHostInfo->HostMachineAddress, &host_machine_address_);
    getSdkVersion(pHostInfo->HostProgramVersion);
    strcpy(pHostInfo->szHostMachineName, "Cortex.dll");

    //TODO Error handling
    return RC_Okay;
}

int CortexMock::exit(){
	// TODO
	return RC_GeneralError;
}

int CortexMock::request(char* szCommand, void** ppResponse, int *pnBytes){
	// TODO
	return RC_GeneralError;
}

sSkyReturn CortexMock::*skyCommand(char *szCommand, int msTimeout){
	// TODO
}

sBodyDefs* CortexMock::getBodyDefs(){
	// TODO
}

int CortexMock::freeBodyDefs(sBodyDefs* pBodyDefs){
	// TODO
	return RC_GeneralError;
}

sFrameOfData* CortexMock::getCurrentFrame(){
	return &current_frame_;
}

int CortexMock::copyFrame(const sFrameOfData* pSrc, sFrameOfData* pDst){
	// TODO
	return RC_GeneralError;
}

int CortexMock::freeFrame(sFrameOfData* pFrame){
	// TODO
	return RC_GeneralError;
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

void CortexMock::extractBodies(sFrameOfData& fod, const rapidjson::Value& parent_value){
	int n_bodies = fod.nBodies;
	for (int i = 0; i < n_bodies; ++i) {
		rapidjson::Value i_body_json = parent_value["bodies"][i];
		sBodyData& i_body_data = fod.BodyData[i];
		i_body_data.szName = i_body_json["name"];
		i_body_data.nMarkers = i_body_json["nMarkers"];
		extractMarkers(i_body_data.Markers, i_body_data.nMarkers, i_body_json["markers"]);
		i_body_data.fAvgMarkerResidual = i_body_json["fAvgMarkerResidual"];
		i_body_data.nSegments = i_body_json["nSegments"];
		extractSegments(i_body_data.Segments, i_body_data.nSegments, i_body_json["segments"]);
		i_body_data.nDofs = i_body_json["nDofs"];

		int n_dofs = i_body_data.nDofs;
		if(n_dofs > 0){
			i_body_data.Dofs = new double[n_dofs];
			for (int i_dof = 0; i_dof < n_dofs; ++i_dof) {
				i_body_data.Dofs[i] = i_body_json["dofs"][i];
			}
		}


		i_body_data.fAvgDofResidual = i_body_json["fAvgDofResidual"];
		i_body_data.nIterations = i_body_json["nIterations"];
		i_body_data.ZoomEncoderValue = i_body_json["encoderZoom"];
		i_body_data.FocusEncoderValue = i_body_json["encoderFocus"];
		i_body_data.IrisEncoderValue = i_body_json["encoderIris"];

		rapidjson::Value cam_track_params = i_body_json["camTrackParams"];
		i_body_data.CamTrackParams[0] = cam_track_params["offsetX"];
		i_body_data.CamTrackParams[1] = cam_track_params["offsetY"];
		i_body_data.CamTrackParams[2] = cam_track_params["offsetZ"];
		i_body_data.CamTrackParams[3] = cam_track_params["offsetAngleX"];
		i_body_data.CamTrackParams[4] = cam_track_params["offsetAngleY"];
		i_body_data.CamTrackParams[5] = cam_track_params["offsetAngleZ"];
		i_body_data.CamTrackParams[6] = cam_track_params["videoWidth"];
		i_body_data.CamTrackParams[7] = cam_track_params["videoHeight"];
		i_body_data.CamTrackParams[8] = cam_track_params["opticalCenterX"];
		i_body_data.CamTrackParams[9] = cam_track_params["opticalCenterY"];
		i_body_data.CamTrackParams[10] = cam_track_params["fovX"];
		i_body_data.CamTrackParams[11] = cam_track_params["fovY"];
		i_body_data.CamTrackParams[12] = cam_track_params["pixelAspect"];
		i_body_data.CamTrackParams[13] = cam_track_params["firstCoefficient"];

		i_body_data.nEvents = i_body_json["nEvents"];

		int n_events = i_body_data.nEvents;
		if(n_events > 0){
			i_body_data.Events = new char*[n_events];
			for (int i_event = 0; i_event < n_events; ++i_event) {
				strcpy(i_body_data.Events[i_event], i_body_json["events"][i].GetString());
			}
		}
	}
}

void CortexMock::extractMarkers(tMarkerData* markers, int n_markers, const rapidjson::Value& parent_value){
	markers = new tMarkerData[n_markers];
	for (int i_marker = 0; i_marker < n_markers; ++i_marker) {
		markers[i_marker][0] = parent_value[i_marker]["x"];
		markers[i_marker][1] = parent_value[i_marker]["y"];
		markers[i_marker][2] = parent_value[i_marker]["z"];
	}
}

void CortexMock::extractAnalogData(sAnalogData& adata, const rapidjson::Value& parent_value){
	int n_channels = adata.nAnalogChannels = parent_value["nAnalogChannels"];
	int n_samples = adata.nAnalogSamples = parent_value["nAnalogSamples"];
	adata.AnalogSamples = new short[n_samples*n_channels];
	int index = 0;
	for (int i_sample=0 ; i_sample<n_samples ; i_sample++)
	{
		for (int i_channel=0 ; i_channel<n_channels ; i_channel++)
		{
			index = i_sample*n_samples+i_channel;
			adata.AnalogSamples[index] = parent_value["analogSamples"][index]["value"];
		}
	}

	int n_force_plates = adata.nForcePlates = parent_value["nForcePlates"];
	int n_force_samples = adata.nForceSamples = parent_value["nForceSamples"];
	adata.Forces = new tForceData[n_force_samples*n_force_plates];
	for (int i_sample=0; i_sample<n_force_samples; i_sample++)
	{
		for (int i_plate=0; i_plate<n_force_plates; i_plate++)
		{
			index = i_sample*n_samples+i_plate;
			auto force_json = parent_value["forces"][index];
			adata.Forces[index][0] = force_json["x"];
			adata.Forces[index][1] = force_json["y"];
			adata.Forces[index][2] = force_json["z"];
			adata.Forces[index][3] = force_json["fX"];
			adata.Forces[index][4] = force_json["fY"];
			adata.Forces[index][5] = force_json["fZ"];
			adata.Forces[index][6] = force_json["mZ"];
		}
	}

	int n_angle_encoders = adata.nAngleEncoders = parent_value["nAngleEncoders"];
	int n_angle_encoder_samples = adata.nAngleEncoderSamples = parent_value["nAngleEncoderSamples"];
	adata.AngleEncoderSamples = new double[n_angle_encoders*n_angle_encoder_samples];
	int index = 0;
	for (int i_sample=0 ; i_sample<n_angle_encoder_samples ; i_sample++)
	{
		for (int i_enc=0 ; i_enc<n_angle_encoders ; i_enc++)
		{
			index = i_sample*n_angle_encoder_samples+i_enc;
			adata.AngleEncoderSamples[index] = parent_value["angleEncoderSamples"][index]["value"];
		}
	}
}

void CortexMock::extractSegments(tSegmentData* segments, int n_segments, const rapidjson::Value& parent_value){
	segments = new tSegmentData[n_segments];
	for (int i_segment = 0; i_segment < n_segments; ++i_segment) {
		segments[i_segment][0] = parent_value[i_segment]["x"];
		segments[i_segment][1] = parent_value[i_segment]["y"];
		segments[i_segment][2] = parent_value[i_segment]["z"];
		segments[i_segment][3] = parent_value[i_segment]["aX"];
		segments[i_segment][4] = parent_value[i_segment]["aY"];
		segments[i_segment][5] = parent_value[i_segment]["aZ"];
		segments[i_segment][6] = parent_value[i_segment]["length"];
	}
}

void freeFrameOfData(sFrameOfData& fod){
	// TODO empty values(?)


	int n_bodies = fod.nBodies;
	for (int i_body = 0; i_body < n_bodies; ++i_body) {
		// free markers
//		int n_markers = fod.BodyData[i_body].nMarkers;
//		for (int i_marker = 0; i_marker < n_markers; ++i_marker) {
//			delete fod.BodyData[i_body].Markers[i_marker]; // TODO is this necessary?
//		}
		delete fod.BodyData[i_body].Markers;

		// free segments
//		int n_segments = fod.BodyData[i_body].nSegments;
//		for (int i_segment = 0; i_segment < n_segments; ++i_segment) {
//			delete fod.BodyData[i_body].Segments[i_segment]; // TODO is this necessary?
//		}
		delete fod.BodyData[i_body].Segments;

		// free dofs
		delete fod.BodyData[i_body].Dofs;
		// free events
		int n_events = fod.BodyData[i_body].nEvents;
		for (int i_event = 0; i_event < n_events; ++i_event) {
			delete fod.BodyData[i_body].Events[i_event];
		}
		delete fod.BodyData[i_body].Events;
	}
	delete fod.BodyData;

	// free uimarkers
//	int n_ui_markers = fod.nUnidentifiedMarkers;
//	for (int i_ui_marker = 0; i_ui_marker < n_ui_markers; ++i_ui_marker) {
//		delete fod.UnidentifiedMarkers[i_ui_marker];
//	}
	delete fod.UnidentifiedMarkers;

	// free analogdata
	delete fod.AnalogData.AnalogSamples;
	delete fod.AnalogData.Forces;
	delete fod.AnalogData.AngleEncoderSamples;
}

void CortexMock::extractFrame(sFrameOfData& fod, int iFrame){
	freeFrameOfData(fod);
	rapidjson::Value frame = document["framesArray"][iFrame];
	fod.iFrame = frame["frame"];
	fod.fDelay = frame["frameDelay"];
	fod.nBodies = frame["nBodies"];

	extractBodies(fod, frame);

	fod.nUnidentifiedMarkers = frame["nUnidentifiedMarkers"];

	extractMarkers(fod.UnidentifiedMarkers, fod.nUnidentifiedMarkers, frame["unidentifiedMarkers"]);

	extractAnalogData(fod.AnalogData, frame["analogData"]);

	rapidjson::Value rc_status = frame["recordingStatus"];
	fod.RecordingStatus.bRecording = rc_status["recording"];
	fod.RecordingStatus.iFirstFrame = rc_status["firstFrame"];
	fod.RecordingStatus.iLastFrame = rc_status["lastFrame"];
	fod.RecordingStatus.szFilename = rc_status["captureFileName"];

	rapidjson::Value tc_value = frame["timeCode"];
	fod.TimeCode.iHours = tc_value["hours"];
	fod.TimeCode.iMinutes = tc_value["minutes"];
	fod.TimeCode.iSeconds = tc_value["seconds"];
	fod.TimeCode.iFrames = tc_value["frames"];
	fod.TimeCode.iStandard = tc_value["standard"];
}

void CortexMock::run(){
	for (int i_frame = 0; i_frame < n_frames; ++i_frame) {
		extractFrame(current_frame_, i_frame);
		dataHandlerFunc(&current_frame_);
	}
}

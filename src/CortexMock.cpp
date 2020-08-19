#include <string.h>
#include <iostream>

#include "CortexMock.hpp"
#include "rapidjson/filereadstream.h"

CortexMock::CortexMock(const std::string capture_file_name):capture_file_name_(capture_file_name) {
	FILE* fp = fopen(capture_file_name_.data(), "r");
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
	inet_aton(szClientsMulticastAddress, &host_machine_address_);

	// establish connection here
	// try {
	//     tcp_connection_ = std::make_unique<kuka_sunrise::TCPConnection>(
	//       inet_ntoa(talk_to_client_address_),
	// 	  talk_to_clients_request_port_,
	//       [this](const std::vector<std::uint8_t> & data) {this->dataRecievedCallback(data);},
	//       [this](const char * server_addr,
	//       const int server_port) {this->connectionLostCallback(server_addr, server_port);});
	//   } catch (...) {
	//     tcp_connection_.reset();
	//   }
	inet_aton(szHostNicCardAddress, &server_.sin_addr);
	server_.sin_family = AF_INET;
  	server_.sin_port = htons(host_port_);
	addrlen = sizeof(server_);
	bind(server_sock_desc_, (struct sockaddr *)&server_, (socklen_t)addrlen);
	listen(server_sock_desc_,10);
	client_sock_desc_ = accept(server_sock_desc_, (struct sockaddr *)&server_, (socklen_t *)&addrlen);

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
    pHostInfo->LatestConfirmationTime = current_framenum_;
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

void CortexMock::freeFrameOfData(sFrameOfData& fod){
	// TODO empty values(?)

	int n_bodies = fod.nBodies;
	if(n_bodies > 0){
		for (int i_body = 0; i_body < n_bodies; ++i_body) {
			sBodyData& i_body_data = fod.BodyData[i_body];
			// free markers
	//		int n_markers = i_body_data.nMarkers;
	//		for (int i_marker = 0; i_marker < n_markers; ++i_marker) {
	//			delete i_body_data.Markers[i_marker]; // TODO is this necessary?
	//		}
			
			if(i_body_data.nMarkers > 0) delete [] i_body_data.Markers;

			// free segments
	//		int n_segments = i_body_data.nSegments;
	//		for (int i_segment = 0; i_segment < n_segments; ++i_segment) {
	//			delete i_body_data.Segments[i_segment]; // TODO is this necessary?
	//		}
			if(i_body_data.nSegments > 0) delete [] i_body_data.Segments;

			// free dofs
			if(i_body_data.nDofs > 0) delete [] i_body_data.Dofs;
			// free events
			int n_events = i_body_data.nEvents;
			for (int i_event = 0; i_event < n_events; ++i_event) {
				if(i_body_data.Events[i_event] != nullptr) delete [] i_body_data.Events[i_event];
			}
			if(i_body_data.nEvents > 0) delete [] i_body_data.Events;
		}
	}

	// free uimarkers
//	int n_ui_markers = fod.nUnidentifiedMarkers;
//	for (int i_ui_marker = 0; i_ui_marker < n_ui_markers; ++i_ui_marker) {
//		delete fod.UnidentifiedMarkers[i_ui_marker];
//	}
	if(fod.nUnidentifiedMarkers > 0) delete [] fod.UnidentifiedMarkers;

	// free analogdata
	if(fod.AnalogData.nAnalogSamples > 0 && fod.AnalogData.nAnalogChannels > 0) delete [] fod.AnalogData.AnalogSamples;
	if(fod.AnalogData.nForceSamples > 0 && fod.AnalogData.nForcePlates > 0) delete [] fod.AnalogData.Forces;
	if(fod.AnalogData.nAngleEncoderSamples > 0 && fod.AnalogData.nAngleEncoders > 0) delete [] fod.AnalogData.AngleEncoderSamples;
}

void CortexMock::extractFrame(sFrameOfData& fod, int iFrame){
	freeFrameOfData(fod);
	const rapidjson::Value& frame = document["framesArray"][iFrame];
	fod.iFrame = frame["frame"].GetInt();
	fod.fDelay = frame["frameDelay"].GetFloat();
	fod.nBodies = frame["nBodies"].GetInt();

	if(fod.nBodies > 0) extractBodies(fod, frame);

	fod.nUnidentifiedMarkers = frame["nUnidentifiedMarkers"].GetInt();

	if(fod.UnidentifiedMarkers > 0){
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

void CortexMock::fodToBytes(const sFrameOfData& fod, std::vector<std::uint8_t> & bytes_data){
	bytes_data.emplace_back(static_cast<std::uint8_t>(fod.iFrame));
	bytes_data.emplace_back(static_cast<std::uint8_t>(fod.nUnidentifiedMarkers));
	// for (int i_ui_marker = 0; i_ui_marker < fod.nUnidentifiedMarkers; i_ui_marker++)
	// {
	// 	bytes_data.emplace_back(static_cast<std::uint8_t>(fod.UnidentifiedMarkers[i_ui_marker][0]));
	// 	bytes_data.emplace_back(static_cast<std::uint8_t>(fod.UnidentifiedMarkers[i_ui_marker][1]));
	// 	bytes_data.emplace_back(static_cast<std::uint8_t>(fod.UnidentifiedMarkers[i_ui_marker][2]));
	// }
}

void CortexMock::dataRecievedCallback(const std::vector<std::uint8_t> & data){
	std::cout << "Data recieved at cortex mock via TCP" << std::endl;
}

void CortexMock::connectionLostCallback(const char * talk_to_host_address, const int talk_to_host_port){
	std::cout << "Connection lost at cortex mock" << std::endl;
}

void CortexMock::dataHandlerFunc(sFrameOfData* p_frame_of_data){
	// std::cout << "Frame " << p_frame_of_data->iFrame << "\tnUnidentifiedMarkers: " << p_frame_of_data->nUnidentifiedMarkers << std::endl;

	// int n_ui_markers = p_frame_of_data->nUnidentifiedMarkers;
	// const rapidjson::Value& frame = document["framesArray"][0];
	// for (int i = 0; i < n_ui_markers; i++)
	// {
	// 	auto i_ui_marker = p_frame_of_data->UnidentifiedMarkers[i];
	// 	std::cout << "UiMarker " << i << ": x:" << i_ui_marker[0] << " y: " << i_ui_marker[1] << " z: " << i_ui_marker[2] << std::endl;
		
	// }
	std::vector<std::uint8_t> bytes_data;
	fodToBytes(*p_frame_of_data, bytes_data);
	write(client_sock_desc_, bytes_data.data(), bytes_data.size());
};

void CortexMock::run(){
	for (int i_frame = 0; i_frame < n_frames; ++i_frame) {
		extractFrame(current_frame_, i_frame);
		dataHandlerFunc(&current_frame_);
	}
}

// int main(int argc, char **argv) {
// 	CortexMock cortex_mock("CaptureWithPlots1.json");
// 	char addr[] = "127.0.0.1";
// 	cortex_mock.initialize(addr, addr);
// 	return 0;
// }

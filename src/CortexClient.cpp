#include <iostream>

#include "CortexClient.hpp"

// CortexClient::CortexClient(std::string server_addr, const int server_port): server_addr_(server_addr), server_port_(server_port){

// }

CortexClient::~CortexClient(){
    freeFrameOfData(current_fod_);
    close(sock);
}

void CortexClient::extractBodies(sFrameOfData& fod, const rapidjson::Value& parent_value){
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

void CortexClient::extractMarkers(tMarkerData* markers, int n_markers, const rapidjson::Value& parent_value){
	for (int i_marker = 0; i_marker < n_markers; ++i_marker) {
		markers[i_marker][0] = parent_value[i_marker]["x"].GetFloat();
		markers[i_marker][1] = parent_value[i_marker]["y"].GetFloat();
		markers[i_marker][2] = parent_value[i_marker]["z"].GetFloat();
	}
}

void CortexClient::extractAnalogData(sAnalogData& adata, const rapidjson::Value& parent_value){
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

void CortexClient::extractSegments(tSegmentData* segments, int n_segments, const rapidjson::Value& parent_value){
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

void CortexClient::freeFrameOfData(sFrameOfData& fod){
	int n_bodies = fod.nBodies;
	if(n_bodies > 0){
		for (int i_body = 0; i_body < n_bodies; ++i_body) {
			sBodyData& i_body_data = fod.BodyData[i_body];
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

	if(fod.nUnidentifiedMarkers > 0) delete [] fod.UnidentifiedMarkers;
	if(fod.AnalogData.nAnalogSamples > 0 && fod.AnalogData.nAnalogChannels > 0) delete [] fod.AnalogData.AnalogSamples;
	if(fod.AnalogData.nForceSamples > 0 && fod.AnalogData.nForcePlates > 0) delete [] fod.AnalogData.Forces;
	if(fod.AnalogData.nAngleEncoderSamples > 0 && fod.AnalogData.nAngleEncoders > 0) delete [] fod.AnalogData.AngleEncoderSamples;
}

void CortexClient::extractFrame(sFrameOfData& fod, const rapidjson::Value& frame){
	freeFrameOfData(fod);
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

void CortexClient::dataReceivedCallback_(char* data){
    rapidjson::StringStream s(data);
    current_frame_json_.ParseStream(s);
    extractFrame(current_fod_, current_frame_json_);
    dataHandlerFunc(current_fod_);
}

void CortexClient::connectionLostCallback_(const char *server_addr, int server_port){
    std::cout << "Connection to server lost" << std::endl;
}

void CortexClient::run(){
	struct sockaddr_in socket_serv_addr;
	char msg_buffer[max_json_frame_size_];
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
    { 
    	std::cout << "Socket creation error" << std::endl; 
    	return; 
    } 
	socket_serv_addr.sin_family = AF_INET; 
    socket_serv_addr.sin_port = htons(server_port_);

	if(inet_pton(AF_INET, server_addr_.data(), &socket_serv_addr.sin_addr)<=0)
    { 
        std::cout << "Invalid address/ Address not supported" << std::endl;
        return; 
    } 
   
    if (connect(sock, (struct sockaddr *)&socket_serv_addr, sizeof(socket_serv_addr)) < 0) 
    {
		std::cout << "Connection Failed" << std::endl;
        return; 
    }

	while(true){
		int length = recv(sock, msg_buffer, max_json_frame_size_, 0);
		if (length > 0){
            msg_buffer[length] = '\0';
			dataReceivedCallback_(msg_buffer);
		}
	}

    // std::unique_ptr<kuka_sunrise::TCPConnection> tcp_connection;
	// try {
	// 	tcp_connection = std::make_unique<kuka_sunrise::TCPConnection>(
	// 	  server_addr_.data(),
	// 	  server_port_,
	// 	  [this](char* data) {dataReceivedCallback_(data);},
	// 	  [this](const char * server_addr,
	// 	  const int server_port) {connectionLostCallback_(server_addr, server_port);});
	// } catch (...) {
	// 	tcp_connection.reset();
	// }
}
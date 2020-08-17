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

void CortexMock::extractBodies(sFrameOfData& fod, rapidjson::Value& frame){

}

void CortexMock::extractMarkers(sFrameOfData& fod, rapidjson::Value& frame){

}

void CortexMock::extractAnalogData(sFrameOfData& fod, rapidjson::Value& frame){

}

void CortexMock::extractFrame(sFrameOfData& fod, int iFrame){
	rapidjson::Value frame = document["framesArray"][iFrame];
	fod.iFrame = frame["frame"];
	fod.fDelay = frame["frameDelay"];
	fod.nBodies = frame["nBodies"];

	//TODO bodies

	fod.nUnidentifiedMarkers = frame["nUnidentifiedMarkers"];

	// TODO markers

	// TODO analog data

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

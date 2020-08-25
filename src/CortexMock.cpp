#include <string.h>
#include <iostream>
#include <chrono>
#include <thread>

#include "CortexMock.hpp"
#include "rapidjson/filereadstream.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

CortexMock::CortexMock(const std::string capture_file_name):capture_file_name_(capture_file_name), server_sock_desc_(socket(AF_INET, SOCK_STREAM, 0)) {
	FILE* fp = fopen(capture_file_name_.data(), "r");
	char read_buffer[65536];
	rapidjson::FileReadStream is(fp, read_buffer, sizeof(read_buffer));

	document.ParseStream(is);
	fclose(fp);
	n_frames = document["framesArray"].Size();
}

CortexMock::~CortexMock(){
	freeFrame(&current_frame_);
	close(server_sock_desc_);
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

	inet_aton(szHostNicCardAddress, &server_.sin_addr);
	server_.sin_family = AF_INET;
  	server_.sin_port = htons(host_port_);
	addrlen = sizeof(server_);
	if(bind(server_sock_desc_, (struct sockaddr *)&server_, (socklen_t)addrlen)<0){
		std::cerr << "Bind failed" << std::endl; 
        return RC_NetworkError;
	}
	if(listen(server_sock_desc_,3) < 0){
		std::cerr << "Listening failed" << std::endl; 
        return RC_NetworkError;
	}
	if((client_sock_desc_ = accept(server_sock_desc_, (struct sockaddr *)&server_, (socklen_t *)&addrlen)) < 0){
		std::cerr << "Accepting failed" << std::endl;
        return RC_NetworkError;
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
	std::cerr << "Connection lost at cortex mock" << std::endl;
}

void CortexMock::dataHandlerFunc(sFrameOfData* p_frame_of_data){
	std::vector<std::uint8_t> bytes_data;
	fodToBytes(*p_frame_of_data, bytes_data);
	send(client_sock_desc_, bytes_data.data(), bytes_data.size(), 0);
	std::cout << "Data of frame " << p_frame_of_data->iFrame << " sent from cortex mock via TCP" << std::endl;
};

void CortexMock::sendFrameJSON(const int i_frame){
	rapidjson::StringBuffer buffer;
	rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
	document["framesArray"][i_frame].Accept(writer);
	send(client_sock_desc_, buffer.GetString(), buffer.GetSize(), 0);
	std::cout << "Data of frame " << i_frame << " sent from cortex mock via TCP, size " << buffer.GetLength() << std::endl;
	std::cout << "Number of Ui Markers: " << document["framesArray"][i_frame]["nUnidentifiedMarkers"].GetInt() << std::endl;
}

void CortexMock::run(){
	for (int i_frame = 0; i_frame < n_frames; ++i_frame) {
		sendFrameJSON(i_frame);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
}

int main(int argc, char **argv) {
	CortexMock cortex_mock("CaptureWithPlots1.json");
	char addr[] = "127.0.0.1";
	cortex_mock.initialize(addr, addr);

	return 0;
}

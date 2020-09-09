#include <chrono>
#include <thread>

#include "CortexClient.hpp"

CortexClient::CortexClient(const std::string& capture_file_name):cortex_mock_(capture_file_name){
	
}

CortexClient::~CortexClient(){
    cortex_mock_.freeFrame(&current_fod_);
}

void CortexClient::run(){
	char* server_addr = new char[server_addr_.length()+1];  // TODO is there a better workaround?
	strcpy(server_addr,server_addr_.data());
	cortex_mock_.initialize(server_addr, server_addr);
	delete [] server_addr;

	std::string forw_comm = "PostForward", backw_comm = "PostBackward",pause_comm = "PostPause";
	char* forw_comm_char = new char[forw_comm.length()+1];
	strcpy(forw_comm_char, forw_comm.data());
	cortex_mock_.request(forw_comm_char, nullptr, nullptr);
	delete [] forw_comm_char;
	std::this_thread::sleep_for(std::chrono::seconds(30));

	char* backw_comm_char = new char[backw_comm.length()+1];
	strcpy(backw_comm_char, backw_comm.data());
	cortex_mock_.request(backw_comm_char, nullptr, nullptr);
	delete [] backw_comm_char;
	std::this_thread::sleep_for(std::chrono::seconds(60));

	char* pause_comm_char = new char[pause_comm.length()+1];
	strcpy(pause_comm_char, pause_comm.data());
	cortex_mock_.request(pause_comm_char, nullptr, nullptr);
	delete [] pause_comm_char;
	std::this_thread::sleep_for(std::chrono::seconds(5));

	cortex_mock_.exit();
}

int CortexClient::setDataHandlerFunc(void (*dataHandlerFunc)(sFrameOfData* pFrameOfData)){
	cortex_mock_.setDataHandlerFunc(dataHandlerFunc);
}

int CortexClient::setErrorMsgHandlerFunc(void (*errorMsgHandlerFunc)(int iLogLevel, char* szLogMessage)){
	cortex_mock_.setErrorMsgHandlerFunc(errorMsgHandlerFunc);
}

int CortexClient::copyFrame(const sFrameOfData* pSrc, sFrameOfData* pDst){
	cortex_mock_.copyFrame(pSrc, pDst);
}
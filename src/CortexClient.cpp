#include <iostream>

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
}

int CortexClient::setdataHandlerFunc(void (*dataHandlerFunc_)(sFrameOfData* pFrameOfData)){
	cortex_mock_.setDataHandlerFunc(dataHandlerFunc_);
}

int CortexClient::copyFrame(const sFrameOfData* pSrc, sFrameOfData* pDst){
	cortex_mock_.copyFrame(pSrc, pDst);
}
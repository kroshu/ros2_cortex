/*
 * MockTest.cpp
 *
 *  Created on: Aug 17, 2020
 *      Author: rosdeveloper
 */

#include <iostream>

#include "CortexMock.hpp"
#include "tcp_connection.hpp"

void dataHandlerFunc(sFrameOfData* p_frame_of_data){
	std::cout << "Frame " << p_frame_of_data->iFrame << "\tBody with name: " << p_frame_of_data->BodyData->szName << std::endl;
}

void connectionLostCallback(char * talk_to_host_address, int talk_to_host_port){
	std::cout << "Connection lost" << std::endl;
}


int main(int argc, char **argv) {
	CortexMock cortex_mock("CaptureWithPlots1.json");
	char* addr = "127.0.0.1";
	int port_num = 3010; // TODO ??
	cortex_mock.setDataHandlerFunc(dataHandlerFunc);

	std::unique_ptr<kuka_sunrise::TCPConnection> tcp_connection;
	try {
		tcp_connection = std::make_unique<kuka_sunrise::TCPConnection>(
		  addr,
		  port_num,
		  [this](sFrameOfData* p_frame_of_data) {dataHandlerFunc(p_frame_of_data);},
		  [this](const char * server_addr,
		  int server_port) {connectionLostCallback(addr,port_num);});
	} catch (...) {
		tcp_connection.reset();
	}
	cortex_mock.setClientCommunicationEnabled(true);
	cortex_mock.initialize(addr, addr);

	return 0;
}

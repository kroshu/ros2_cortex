/*
 * MockTest.cpp
 *
 *  Created on: Aug 17, 2020
 *      Author: Gergely Kovács
 */

#include <iostream>

#include "CortexMock.hpp"
#include "tcp_connection.hpp"

// void dataHandlerFunc(sFrameOfData* p_frame_of_data){
// 	std::cout << "Frame " << p_frame_of_data->iFrame << "\tnUnidentifiedMarkers: " << p_frame_of_data->nUnidentifiedMarkers << std::endl;
// }

void bytesToFod(const std::vector<std::uint8_t> & data, sFrameOfData& p_frame_of_data){
	int i_frame = static_cast<int>(data[0]);
	int n_ui_markers = static_cast<int>(data[1]);

	std::cout << "Frame " << i_frame << "\tnUnidentifiedMarkers: " << n_ui_markers << std::endl;
	// for (int i = 0; i < n_ui_markers; i++)
	// {
	// 	tMarkerData i_ui_marker = {data[1+i*3], data[2+i*3], data[3+i*3]};
	// 	std::cout << "UiMarker " << i << ": x:" << i_ui_marker[0] << " y: " << i_ui_marker[1] << " z: " << i_ui_marker[2] << std::endl;
	// }
}

void dataReceivedCallback(char* data){
	std::cout << "Data recieved via TCP at mock test" << std::endl;
	std::string frame_str(data);
	std::cout << frame_str << std::endl;
	// sFrameOfData fod;
	// bytesToFod(data, fod);
}

void connectionLostCallback(const char * talk_to_host_address, const int talk_to_host_port){
	std::cout << "Connection lost at mock test" << std::endl;
}


int main(int argc, char **argv) {
	//CortexMock cortex_mock("CaptureWithPlots1.json");
	char addr[] = "127.0.0.1";
	const int port_num = 30001; // TODO ??
	// cortex_mock.setDataHandlerFunc(dataHandlerFunc);
	// cortex_mock.setClientCommunicationEnabled(true);
	// cortex_mock.initialize(addr, addr);

	// std::unique_ptr<kuka_sunrise::TCPConnection> tcp_connection;
	// try {
	// 	tcp_connection = std::make_unique<kuka_sunrise::TCPConnection>(
	// 	  addr,
	// 	  port_num,
	// 	  [](const std::vector<std::uint8_t> & data) {dataRecievedCallback(data);},
	// 	  [](const char * server_addr,
	// 	  const int server_port) {connectionLostCallback(server_addr, server_port);});
	// } catch (...) {
	// 	tcp_connection.reset();
	// }

	

	return 0;
}

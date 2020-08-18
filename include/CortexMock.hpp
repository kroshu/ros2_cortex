#ifndef CORTEX_MOCK_HPP_
#define CORTEX_MOCK_HPP_

#include <string>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <memory>

#include "tcp_connection.hpp"
#include "Cortex.h"
#include "rapidjson/document.h"

class CortexMock{
    public:
        CortexMock(const std::string captureFileName);
        ~CortexMock();
        int getSdkVersion(unsigned char Version[4]);
        int setVerbosityLevel(int iLevel);
        int getVerbosityLevel();
        int setMinTimeout(int msTimeout);
        int getMinTimeout();
        int setErrorMsgHandlerFunc(void (*dataHandlerFunc)(int iLogLevel, char* szLogMessage));
        int setDataHandlerFunc(void (*dataHandlerFunc)(sFrameOfData* pFrameOfData));
        int sendDataToClients(sFrameOfData* pFrameOfData);
        void setClientCommunicationEnabled(int bEnabled);
        int isClientCommunicationEnabled();
        void setThreadPriorities(maThreadPriority ListenForHost, maThreadPriority ListenForData, maThreadPriority ListenForClients);
        int configurePortNumbers(int TalkToHostPort,                 //0 == find available
										int HostPort,
										int HostMulticastPort, 
										int TalkToClientsRequestPort = 0,   //0 == find available
										int TalkToClientsMulticastPort = 0, //0 == find available
										int ClientsMulticastPort = -1);
        int initialize(	char* szTalkToHostNicCardAddress,
							char* szHostNicCardAddress,
							char* szHostMulticastAddress = (char*)"225.1.1.1",
							char* szTalkToClientsNicCardAddress = 0,
							char* szClientsMulticastAddress = (char*)"225.1.1.2");
        int getPortNumbers(	int *TalkToHostPort,
								int *HostPort, 
								int *HostMulticastPort, 
								int *TalkToClientsRequestPort,
								int *TalkToClientsMulticastPort,
								int *ClientsMulticastPort);
        int getAddresses(char* szTalkToHostNicCardAddress,
							char* szHostNicCardAddress,
							char* szHostMulticastAddress,
							char* szTalkToClientsNicCardAddress,
							char* szClientsMulticastAddress);
        int getHostInfo(sHostInfo *pHostInfo);
        int exit();
        int request(char* szCommand, void** ppResponse, int *pnBytes);
        sSkyReturn* skyCommand(char *szCommand, int msTimeout);
        sBodyDefs* getBodyDefs();
        int freeBodyDefs(sBodyDefs* pBodyDefs);
        sFrameOfData* getCurrentFrame();
        int copyFrame(const sFrameOfData* pSrc, sFrameOfData* pDst);
        int freeFrame(sFrameOfData* pFrame);
        int sendHtr(sHierarchy *pHierarchy, tSegmentData *pFrame);
        int setMetered(bool bActive, float fFixedLatency);
        void constructRotationMatrix(double angles[3], int iRotationOrder, double matrix[3][3]);
        void extractEulerAngles(double matrix[3][3],int iRotationOrder, double angles[3]);

    private:
        int min_time_out_=500, n_frames;
        in_addr host_machine_address_, host_multicast_address_, talk_to_host_address_, talk_to_client_address_, client_multicast_address_;
        const std::string capture_file_name_;
        int talk_to_host_port_ = 0, host_port_ = 1510, host_multicast_port_ = 1001;
        int talk_to_clients_request_port_ = 0, talk_to_clients_multicast_port_ = 0, clients_multicast_port_ = -1;
        std::unique_ptr<kuka_sunrise::TCPConnection> tcp_connection_;
        rapidjson::Document document;
        sFrameOfData current_frame_;
        void dataHandlerFunc(sFrameOfData* pFrameOfData);
        void connectionLostCallback(in_addr * talk_to_client_address, int talk_to_clients_request_port);
        void run();
        void extractFrame(sFrameOfData& fod, int iFrame);
        void extractBodies(sFrameOfData& fod, const rapidjson::Value& parent_value);
        void extractMarkers(tMarkerData* markers, int n_markers, const rapidjson::Value& parent_value);
        void extractAnalogData(sAnalogData& adata, const rapidjson::Value& parent_value);
        void extractSegments(tSegmentData* segments, int n_segments, const rapidjson::Value& parent_value);
        void freeFrameOfData(sFrameOfData& fod);
};

#endif

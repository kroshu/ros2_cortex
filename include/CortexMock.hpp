#ifndef CORTEX_MOCK_HPP_
#define CORTEX_MOCK_HPP_

#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <memory>
#include <functional>
#include <map>

#include "Cortex.h"
#include "rapidjson/document.h"

// TODO consistent style

class CortexMock{
    public:
        CortexMock(const std::string& captureFileName);
        // CortexMock(const CortexMock&);
        ~CortexMock();
        int getSdkVersion(unsigned char Version[4]);
        int setVerbosityLevel(int iLevel);
        int getVerbosityLevel();
        int setMinTimeout(int msTimeout);
        int getMinTimeout();
        int setErrorMsgHandlerFunc(void (*errorMsgHandlerFunc)(int iLogLevel, char* szLogMessage));
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
							char* szTalkToClientsNicCardAddress = (char*)"127.0.0.1",
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

        void getCaptureFilename(std::string& dest) const;

    private:
        int min_time_out_ = 500, n_frames_, current_framenum_ = 0;
        int verbosity_level_ = 2, frame_rate_ = 200;
        bool client_comm_enabled_ = false;
        pthread_t run_thread_;
        enum class PlayMode {paused=-1, forwards, backwards};
        PlayMode play_mode_ = PlayMode::paused;
        enum class Request {LiveMode, Pause, SetOutputName, StartRecording,
                            StopRecording, ResetIDs, PostForward, PostBackward, PostPause,
                            PostGetPlayMode, GetContextFrameRate, GetContextAnalogSampleRate,
                            GetContextAnalogBitDepth, GetUpAxis, GetConversionToMillimeters,
                            GetFrameOfData};
        std::map<std::string, Request> map_string_to_request = {
            {"LiveMode", Request::LiveMode},
            {"Pause", Request::Pause},
            {"SetOutputName", Request::SetOutputName},
            {"StartRecording", Request::StartRecording},
            {"StopRecording", Request::StopRecording},
            {"ResetIDs", Request::ResetIDs},
            {"PostForward", Request::PostForward},
            {"PostBackward", Request::PostBackward},
            {"PostPause", Request::PostPause},
            {"PostGetPlayMode", Request::PostGetPlayMode},
            {"GetContextFrameRate", Request::GetContextFrameRate},
            {"GetContextAnalogSampleRate", Request::GetContextAnalogSampleRate},
            {"GetContextAnalogBitDepth", Request::GetContextAnalogBitDepth},
            {"GetUpAxis", Request::GetUpAxis},
            {"GetConversionToMillimeters", Request::GetConversionToMillimeters},
            {"GetFrameOfData", Request::GetFrameOfData}
        };
        in_addr host_machine_address_, host_multicast_address_, talk_to_host_address_, talk_to_client_address_, client_multicast_address_;
        std::string capture_file_name_;
        int talk_to_host_port_ = 30000, host_port_ = 30001, host_multicast_port_ = 30002;
        int talk_to_clients_request_port_ = 30003, talk_to_clients_multicast_port_ = 30004, clients_multicast_port_ = 30005;
        rapidjson::Document document_;
        sFrameOfData current_frame_;
        std::function<void(sFrameOfData*)> dataHandlerFunc_;
        std::function<void(int iLogLevel, char* szLogMessage)> errorMsgHandlerFunc_;
        static void* run_helper(void* cortex_mock);
        void run();
        void extractFrame(sFrameOfData& fod, int iFrame);
        void extractBodies(sFrameOfData& fod, const rapidjson::Value& parent_value);
        void extractMarkers(tMarkerData* markers, int n_markers, const rapidjson::Value& parent_value);
        void extractAnalogData(sAnalogData& adata, const rapidjson::Value& parent_value);
        void extractSegments(tSegmentData* segments, int n_segments, const rapidjson::Value& parent_value);
        void initReadFile();
};

#endif

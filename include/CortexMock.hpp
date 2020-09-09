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

// TODO consistent style - rename params

class CortexMock{
    public:
        CortexMock(const std::string& captureFileName);
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
        int n_frames_, current_framenum_ = 0, verbosity_level_ = 2, analog_bit_depth_= 16; // a_b_d_ 12 or 16 usually
        bool running_ = false;
        float conv_rate_to_mm_ = 1.0, frame_rate_ = 200.0, analog_sample_rate_ = 600.0;
        pthread_t run_thread_;
        enum class PlayMode {paused=-1, forwards, backwards};
        int play_mode_ = static_cast<int>(PlayMode::paused);
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
        enum class Axis {x=0, y, z};
        int axis_up_ = static_cast<int>(Axis::z);
        in_addr host_machine_address_, host_multicast_address_, talk_to_host_address_, talk_to_client_address_, client_multicast_address_;
        std::string capture_file_name_;
        int talk_to_host_port_ = 30000, host_port_ = 30001, host_multicast_port_ = 30002;
        int talk_to_clients_request_port_ = 30003, talk_to_clients_multicast_port_ = 30004, clients_multicast_port_ = 30005;
        rapidjson::Document document_;
        sFrameOfData current_frame_;
        sBodyDefs body_defs_;
        std::function<void(sFrameOfData*)> dataHandlerFunc_;
        std::function<void(int iLogLevel, char* szLogMessage)> errorMsgHandlerFunc_;
        static void* run_helper(void* cortex_mock);
        void run();
        void extractFrame(sFrameOfData& fod, int iFrame);
        void extractBodies(sFrameOfData& fod, const rapidjson::Value& parent_frame_json);
        // TODO check if passing vector would be better here
        void extractMarkers(tMarkerData* markers, int n_markers, const rapidjson::Value& markers_json);
        void extractAnalogData(sAnalogData& adata, const rapidjson::Value& analog_data_json);
        // TODO check if passing vector would be better here
        void extractSegments(tSegmentData* segments, int n_segments, const rapidjson::Value& segments_json);
        void extractBodyDefs(sBodyDefs& body_defs, const rapidjson::Value& body_defs_json);
        void extractBodyDef(sBodyDef& body_def, const rapidjson::Value& body_def_json);
        void initReadFile();
        void errorMsgInString(int i_level, std::string msg);
};

#endif

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

namespace ros2_cortex{

class CortexMock{
    public:
        CortexMock(std::string& capture_file_name);
        CortexMock(const CortexMock& other);
        void swap(CortexMock& other) throw();
        CortexMock& operator=(CortexMock other);
        ~CortexMock();
        int getSdkVersion(unsigned char version[4]);
        int setVerbosityLevel(int i_level);
        int getVerbosityLevel();
        int setMinTimeout(int ms_timeout);
        int getMinTimeout();
        int setErrorMsgHandlerFunc(void (*errorMsgHandlerFunc)(int i_log_level, char* sz_log_message));
        int setDataHandlerFunc(void (*dataHandlerFunc)(sFrameOfData* p_frame_of_data));
        int sendDataToClients(sFrameOfData* p_frame_of_data);
        void setClientCommunicationEnabled(int b_enabled);
        int isClientCommunicationEnabled();
        void setThreadPriorities(maThreadPriority listen_for_host, maThreadPriority listen_for_data, maThreadPriority listen_for_clients);
        int configurePortNumbers(int talk_to_host_port,                 //0 == find available
										int host_port, 
										int host_multicast_port, 
										int talk_to_clients_request_port = 0,   //0 == find available
										int talk_to_clients_multicast_port = 0,   //0 == find available
										int clients_multicast_port = -1);
        int initialize(char* sz_talk_to_host_nic_card_address,
							char* sz_host_nic_card_address,
							char* sz_host_multicast_address = nullptr,
							char* sz_talk_to_clients_nic_card_address = nullptr,
							char* sz_clients_multicast_address = nullptr);
        int getPortNumbers(int *talk_to_host_port,
								int *host_port, 
								int *host_multicast_port, 
								int *talk_to_clients_request_port,
								int *talk_to_clients_multicast_port,
								int *clients_multicast_port);
        int getAddresses(char* sz_talk_to_host_nic_card_address,
							char* sz_host_nic_card_address,
							char* sz_host_multicast_address,
							char* sz_talk_to_clients_nic_card_address,
							char* sz_clients_multicast_address);
        int getHostInfo(sHostInfo *p_host_info);
        int exit();
        int request(char* sz_command, void** pp_response, int *pn_bytes);
        sSkyReturn* skyCommand(char *sz_command, int ms_timeout);
        sBodyDefs* getBodyDefs();
        int freeBodyDefs(sBodyDefs* p_body_defs);
        sFrameOfData* getCurrentFrame();
        int copyFrame(const sFrameOfData* p_src, sFrameOfData* p_dst);
        int freeFrame(sFrameOfData* p_frame);
        int sendHtr(sHierarchy *p_hierarchy, tSegmentData *p_frame);
        int setMetered(bool b_active, float f_fixed_latency);
        void constructRotationMatrix(double angles[3], int i_rotation_order, double matrix[3][3]);
        void extractEulerAngles(double matrix[3][3],int i_rotation_order, double angles[3]);

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

}

#endif

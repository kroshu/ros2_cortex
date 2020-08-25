#ifndef CORTEX_CLIENT_HPP_
#define CORTEX_CLIENT_HPP_

#include <condition_variable>

#include "CortexMock.hpp"

class CortexClient{
public:
    // explicit CortexClient(std::string server_addr, const int server_port);
    void run();
    ~CortexClient();
private:
    void dataReceivedCallback_(char* data);
    void connectionLostCallback_(const char *server_addr, int server_port);
    void extractFrame(sFrameOfData& fod, const rapidjson::Value& frame);
    void extractBodies(sFrameOfData& fod, const rapidjson::Value& parent_value);
    void extractMarkers(tMarkerData* markers, int n_markers, const rapidjson::Value& parent_value);
    void extractAnalogData(sAnalogData& adata, const rapidjson::Value& parent_value);
    void extractSegments(tSegmentData* segments, int n_segments, const rapidjson::Value& parent_value);
    void freeFrameOfData(sFrameOfData& fod);
    bool connect();
    bool isConnected();
    rapidjson::Document current_frame_json_; // TODO initialize
    sFrameOfData current_fod_; // TODO initialize
    const std::string server_addr_ = "127.0.0.1";
    const int server_port_ = 30001;
    const int max_json_frame_size_ = 10000;
    int sock = 0;
    std::unique_ptr<kuka_sunrise::TCPConnection> tcp_connection_;
    std::mutex m_;
    std::condition_variable cv_;
protected:
    virtual void dataHandlerFunc(sFrameOfData& fod)=0;
};

#endif
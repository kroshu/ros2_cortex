#ifndef CORTEX_CLIENT_HPP_
#define CORTEX_CLIENT_HPP_

#include <condition_variable>

#include "CortexMock.hpp"

// TODO consistent style

class CortexClient{
public:
    explicit CortexClient(const std::string& capture_file_name);
    void run();
    ~CortexClient();
private:
    void freeFrameOfData(sFrameOfData& fod);
    bool connect();
    bool isConnected();
    sFrameOfData current_fod_; // TODO initialize
    const std::string server_addr_ = "127.0.0.1";
    const int server_port_ = 30001;
    const int max_json_frame_size_ = 10000;
    int sock = 0;
    std::mutex m_;
    std::condition_variable cv_;
protected:
    virtual void dataHandlerFunc_(sFrameOfData* fod)=0;
    CortexMock cortex_mock_;
};

#endif
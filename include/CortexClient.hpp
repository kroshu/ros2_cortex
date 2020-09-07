#ifndef CORTEX_CLIENT_HPP_
#define CORTEX_CLIENT_HPP_

#include <condition_variable>
#include <functional>

#include "CortexMock.hpp"

// TODO consistent style

class CortexClient{
public:
    explicit CortexClient(const std::string& capture_file_name);
    void run();
    ~CortexClient();
    virtual void dataHandlerFunc_(sFrameOfData*) = 0;
    int setdataHandlerFunc(void (*)(sFrameOfData*));
    int copyFrame(const sFrameOfData* pSrc, sFrameOfData* pDst);
private:
    sFrameOfData current_fod_; // TODO initialize
    const std::string server_addr_ = "127.0.0.1"; // TODO make const and string
    // const int server_port_ = 30001;
protected:
    CortexMock cortex_mock_;
};

#endif
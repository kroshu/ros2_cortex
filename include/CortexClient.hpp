#ifndef CORTEX_CLIENT_HPP_
#define CORTEX_CLIENT_HPP_

#include <condition_variable>
#include <functional>

#include "CortexMock.hpp"

// TODO consistent style

class CortexClient{
public:
    explicit CortexClient(const std::string&);
    void run();
    ~CortexClient();
    virtual void dataHandlerFunc_(sFrameOfData*) = 0;
    virtual void errorMsgHandlerFunc_(int, char*) = 0;
    int setDataHandlerFunc(void (*)(sFrameOfData*));
    int setErrorMsgHandlerFunc(void (*)(int, char*));
    int copyFrame(const sFrameOfData*, sFrameOfData*);
private:
    const std::string server_addr_ = "127.0.0.1"; // TODO make const and string
    // const int server_port_ = 30001;
protected:
    CortexMock cortex_mock_;
    sFrameOfData current_fod_;  // TODO initialize
};

#endif
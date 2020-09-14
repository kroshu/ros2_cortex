#ifndef CORTEX_CLIENT_HPP_
#define CORTEX_CLIENT_HPP_

#include <condition_variable>
#include <functional>

#include "CortexMock.hpp"

class CortexClient{
public:
    explicit CortexClient(const std::string& capture_file_name);
    void run();
    ~CortexClient();
    virtual void dataHandlerFunc_(sFrameOfData* p_frame_of_data) = 0;
    virtual void errorMsgHandlerFunc_(int i_log_level, char* sz_log_message) = 0;
    int setDataHandlerFunc(void (*dataHandlerFunc)(sFrameOfData* p_frame_of_data));
    int setErrorMsgHandlerFunc(void (*errorMsgHandlerFunc)(int i_log_level, char* sz_log_message));
    int copyFrame(const sFrameOfData* p_src, sFrameOfData* p_dst);
private:
    std::string server_addr_ = "127.0.0.1";
protected:
    CortexMock cortex_mock_;
    sFrameOfData current_fod_;
};

#endif
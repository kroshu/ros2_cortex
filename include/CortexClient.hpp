#ifndef CORTEX_CLIENT_HPP_
#define CORTEX_CLIENT_HPP_

#include <condition_variable>
#include <functional>

#include "CortexMock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class CortexClient: public rclcpp_lifecycle::LifecycleNode {
public:
    explicit CortexClient(const std::string& capture_file_name);

    ~CortexClient();
    virtual void dataHandlerFunc_(sFrameOfData* p_frame_of_data) = 0;
    virtual void errorMsgHandlerFunc_(int i_log_level, char* sz_log_message) = 0;
    int setDataHandlerFunc(void (*dataHandlerFunc)(sFrameOfData* p_frame_of_data));
    int setErrorMsgHandlerFunc(void (*errorMsgHandlerFunc)(int i_log_level, char* sz_log_message));
    int copyFrame(const sFrameOfData* p_src, sFrameOfData* p_dst);

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state);

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state);

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & state);

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state);

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state);

    virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_error(const rclcpp_lifecycle::State & state);
private:
    std::string server_addr_ = "127.0.0.1";
    void run();
    static void* run_helper(void* cortex_client);
    void exit();
    std::unique_ptr<pthread_t> client_thread_;

protected:
    CortexMock cortex_mock_;
    sFrameOfData current_fod_;

};

#endif

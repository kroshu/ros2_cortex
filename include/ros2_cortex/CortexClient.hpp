#ifndef CORTEX_CLIENT_HPP_
#define CORTEX_CLIENT_HPP_

#include <condition_variable>
#include <functional>
#include "lifecycle_msgs/msg/state.hpp"

#include "ros2_cortex/CortexMock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_cortex{

struct ParameterSetAccessRights
{
  bool unconfigured;
  bool inactive;
  bool active;
  bool finalized;
  bool isSetAllowed(std::uint8_t current_state) const
  {
    switch (current_state)
    {
      case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
        return unconfigured;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
        return inactive;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
        return active;
      case lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED:
        return finalized;
      default:
        return false;
    }
  }
};

class CortexClient: public rclcpp_lifecycle::LifecycleNode {
public:
    explicit CortexClient(const std::string& node_name);
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
    rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &parameters);
	bool canSetParameter(const rclcpp::Parameter &param);
	bool onCapFileNameChangeRequest(const rclcpp::Parameter& param);
	bool onRequestCommandChanged(const rclcpp::Parameter& param);
	std::map<std::string, struct ParameterSetAccessRights> parameter_set_access_rights_;
	// fs::path capture_file_path_;
	std::string capture_file_path_;
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS =
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn ERROR =
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

protected:
    CortexMock cortex_mock_;
    sFrameOfData current_fod_;

};
}

#endif

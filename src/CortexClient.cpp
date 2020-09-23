#include <chrono>
#include <thread>
#include <iostream>

#include "ros2_cortex/CortexClient.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_cortex{

std::string getFileExtension(const std::string& file_name)
{
    if(file_name.find_last_of(".") != std::string::npos)
        return file_name.substr(file_name.find_last_of(".")+1);
    return "";
}

CortexClient::CortexClient(const std::string& node_name):rclcpp_lifecycle::LifecycleNode(node_name){
	this->declare_parameter("capture_file_path", rclcpp::ParameterValue("/home/rosdeveloper/ros2_ws/src/ros2_cortex/CaptureWithPlots1.json"));
	this->set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter> &parameters)
	{ return this->onParamChange(parameters);});
	parameter_set_access_rights_.emplace("capture_file_path", ParameterSetAccessRights {true, true, false, false});
}

CortexClient::~CortexClient(){
    cortex_mock_.freeFrame(&current_fod_);
    cortex_mock_.exit();
}

void CortexClient::exit(){
	cortex_mock_.freeFrame(&current_fod_);
	cortex_mock_.exit();
}

void* CortexClient::run_helper(void* cortex_client){
    static_cast<CortexClient*>(cortex_client)->run();
	return nullptr;
}

void CortexClient::run(){
	cortex_mock_.initialize(&server_addr_[0], &server_addr_[0]);

	std::string forw_comm = "PostForward", backw_comm = "PostBackward", pause_comm = "PostPause";
	cortex_mock_.request(&forw_comm[0], nullptr, nullptr);
	std::this_thread::sleep_for(std::chrono::seconds(30));

	cortex_mock_.request(&backw_comm[0], nullptr, nullptr);
	std::this_thread::sleep_for(std::chrono::seconds(60));

	cortex_mock_.request(&pause_comm[0], nullptr, nullptr);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	std::string rec_comm = "StartRecording";
	cortex_mock_.request(&rec_comm[0], nullptr, nullptr);
	std::this_thread::sleep_for(std::chrono::seconds(5));

	std::string fps_comm = "GetContextFrameRate";
	void *p_response = nullptr;
	cortex_mock_.request(&fps_comm[0], &p_response, nullptr);
	float frame_rate = *static_cast<float*>(p_response);
	std::cout << "Frame rate: " << frame_rate << std::endl;
	std::this_thread::sleep_for(std::chrono::seconds(5));
}

int CortexClient::setDataHandlerFunc(void (*dataHandlerFunc)(sFrameOfData* p_frame_of_data)){
	return cortex_mock_.setDataHandlerFunc(dataHandlerFunc);
}

int CortexClient::setErrorMsgHandlerFunc(void (*errorMsgHandlerFunc)(int i_log_level, char* sz_log_message)){
	return cortex_mock_.setErrorMsgHandlerFunc(errorMsgHandlerFunc);
}

int CortexClient::copyFrame(const sFrameOfData* p_src, sFrameOfData* p_dst){
	return cortex_mock_.copyFrame(p_src, p_dst);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_configure(const rclcpp_lifecycle::State & state){
	// TODO implement what is needed
	return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_cleanup(const rclcpp_lifecycle::State & state){
	// TODO implement what is needed
	return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_shutdown(const rclcpp_lifecycle::State & state){
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result = SUCCESS;
	switch (state.id()) {
		case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
			result = this->on_deactivate(get_current_state());
			if (result != SUCCESS) {
				break;
			}
			result = this->on_cleanup(get_current_state());
			break;
		case lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE:
			result = this->on_cleanup(get_current_state());
			break;
		case lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED:
			break;
		default:
			break;
	}
	return result;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_activate(const rclcpp_lifecycle::State & state){
	cortex_mock_ = capture_file_path_;
	client_thread_ = std::make_unique<pthread_t>();
	if (pthread_create(client_thread_.get(), nullptr, &CortexClient::run_helper, this)) {
	    RCLCPP_ERROR(get_logger(), "pthread_create error");
	    RCLCPP_ERROR(get_logger(), std::strerror(errno));
	    return ERROR;
	}
	return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_deactivate(const rclcpp_lifecycle::State & state){
	exit();
	return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_error(const rclcpp_lifecycle::State & state){
	RCLCPP_INFO(get_logger(), "An error occured");
	return SUCCESS;
}

rcl_interfaces::msg::SetParametersResult CortexClient::onParamChange(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const rclcpp::Parameter &param : parameters)
  {
    if (param.get_name() == "capture_file_path" && canSetParameter(param))
    {
      result.successful = onCapFileNameChangeRequest(param);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid parameter name %s", param.get_name().c_str());
    }
  }
  return result;
}

bool CortexClient::canSetParameter(const rclcpp::Parameter &param)
{
  try
  {
    if (!parameter_set_access_rights_.at(param.get_name()).isSetAllowed(this->get_current_state().id()))
    {
      RCLCPP_ERROR(this->get_logger(), "Parameter %s cannot be changed while in state %s",
                   param.get_name().c_str(), this->get_current_state().label().c_str());
      return false;
    }
  }
  catch (const std::out_of_range &e)
  {
    RCLCPP_ERROR(this->get_logger(),
                 "Parameter set access rights for parameter %s couldn't be determined", param.get_name().c_str());
    return false;
  }
  return true;
}

bool CortexClient::onCapFileNameChangeRequest(const rclcpp::Parameter& param)
{
	if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
	{
		 RCLCPP_ERROR(this->get_logger(), "Invalid parameter type for parameter %s",
					  param.get_name().c_str());
		 return false;
	}

	// fs::path temp_path = param.as_string();
	std::string temp_path = param.as_string();

	//if (temp_path.extension() != "json")
	if(getFileExtension(temp_path) != "json")
	{
		 RCLCPP_ERROR(this->get_logger(), "Invalid file format for parameter %s",
					  param.get_name().c_str());
		 return false;
	}
	capture_file_path_ = temp_path;
	cortex_mock_ = capture_file_path_);
	return true;
}

}

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

CortexClient::CortexClient(const std::string& node_name):rclcpp_lifecycle::LifecycleNode(node_name),
		capture_file_path_("/home/rosdeveloper/ros2_ws/src/ros2_cortex/CaptureWithPlots1.json"),
		cortex_mock_(capture_file_path_){
	this->declare_parameter("capture_file_path", rclcpp::ParameterValue(capture_file_path_));
	parameter_set_access_rights_.emplace("capture_file_path", ParameterSetAccessRights {true, true, false, false});

	std::string forw_comm = "PostForward";
	this->declare_parameter("request_command", rclcpp::ParameterValue(forw_comm));
	parameter_set_access_rights_.emplace("request_command", ParameterSetAccessRights {false, false, true, false});

	this->set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter> &parameters)
		{ return this->onParamChange(parameters);});
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

	std::string req_comm = this->get_parameter("request_command").as_string();
	cortex_mock_.request(&req_comm[0], nullptr, nullptr);
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
	return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_cleanup(const rclcpp_lifecycle::State & state){
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
    else if (param.get_name() == "request_command" && canSetParameter(param))
    {
      result.successful = onRequestCommandChanged(param);
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

	std::string temp_path = param.as_string();

	if(getFileExtension(temp_path) != "json")
	{
		 RCLCPP_ERROR(this->get_logger(), "Invalid file format for parameter %s",
					  param.get_name().c_str());
		 return false;
	}
	capture_file_path_ = temp_path;
	cortex_mock_ = CortexMock(capture_file_path_);
	return true;
}

bool CortexClient::onRequestCommandChanged(const rclcpp::Parameter& param){
	if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_STRING)
	{
		 RCLCPP_ERROR(this->get_logger(), "Invalid parameter type for parameter %s",
					  param.get_name().c_str());
		 return false;
	}

	std::string req_comm = param.as_string();
	void *p_response = nullptr;
	cortex_mock_.request(&req_comm[0], &p_response, nullptr);
	if(req_comm == "PostGetPlayMode" || req_comm == "GetContextAnalogBitDepth" ||
			req_comm == "GetUpAxis")
		RCLCPP_INFO(get_logger(), "Result of request " + req_comm + ": " + std::to_string(*static_cast<int*>(p_response)));
	else if(req_comm == "GetContextFrameRate" || req_comm == "GetContextAnalogSampleRate" ||
			req_comm == "GetConversionToMillimeters")
		RCLCPP_INFO(get_logger(), "Result of request " + req_comm + ": " + std::to_string(*static_cast<float*>(p_response)));
	else if(req_comm == "GetFrameOfData"){
		sFrameOfData fod;
		cortex_mock_.copyFrame(static_cast<sFrameOfData*>(p_response), &fod);
		RCLCPP_INFO(get_logger(), "Frame " +std::to_string(fod.iFrame));
		RCLCPP_INFO(get_logger(), "Number of unidentified markers " + std::to_string(fod.nUnidentifiedMarkers));
	}

	return true;
}

}

#include <chrono>
#include <thread>
#include <iostream>

#include "CortexClient.hpp"

CortexClient::CortexClient(const std::string& capture_file_name):rclcpp_lifecycle::LifecycleNode("cortex_client"), cortex_mock_(capture_file_name){
	marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
}

CortexClient::~CortexClient(){
    cortex_mock_.freeFrame(&current_fod_);
}

void CortexClient::run(){
	cortex_mock_.initialize(&server_addr_[0], &server_addr_[0]);

	std::string forw_comm = "PostForward", backw_comm = "PostBackward",pause_comm = "PostPause";
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

	cortex_mock_.exit();
}

int CortexClient::setDataHandlerFunc(void (*dataHandlerFunc)(sFrameOfData* p_frame_of_data)){
	cortex_mock_.setDataHandlerFunc(dataHandlerFunc);
}

int CortexClient::setErrorMsgHandlerFunc(void (*errorMsgHandlerFunc)(int i_log_level, char* sz_log_message)){
	cortex_mock_.setErrorMsgHandlerFunc(errorMsgHandlerFunc);
}

int CortexClient::copyFrame(const sFrameOfData* p_src, sFrameOfData* p_dst){
	cortex_mock_.copyFrame(p_src, p_dst);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_configure(const rclcpp_lifecycle::State & state){

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_cleanup(const rclcpp_lifecycle::State & state){

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_shutdown(const rclcpp_lifecycle::State & state){

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_activate(const rclcpp_lifecycle::State & state){

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_deactivate(const rclcpp_lifecycle::State & state){

}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CortexClient::on_error(const rclcpp_lifecycle::State & state){

}

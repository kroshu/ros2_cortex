#include <memory>

#include "MotionTracker.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

MotionTracker::MotionTracker(): rclcpp_lifecycle::LifecycleNode("motion_tracker"), qos(rclcpp::QoS(rclcpp::KeepLast(1))){
	qos.best_effort();
	msg_strategy = std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<visualization_msgs::msg::MarkerArray>>();
	callback = [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) -> void
				{markersReceivedCallback(msg);};
}

void MotionTracker::markersReceivedCallback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg){
	for(auto it = msg->markers.begin(); it != msg->markers.end(); ++it){
		RCLCPP_INFO(get_logger(), "Namespace: " + it->ns);
		RCLCPP_INFO(get_logger(), "ID: " + std::to_string(it->id));

		RCLCPP_INFO(get_logger(), "Point: x: " + std::to_string(it->pose.position.x) +
				" y: " + std::to_string(it->pose.position.y) +
				" z: " + std::to_string(it->pose.position.z));
	}
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_configure(const rclcpp_lifecycle::State & state){
	// TODO implement what is needed
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_cleanup(const rclcpp_lifecycle::State & state){
	// TODO implement what is needed
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_shutdown(const rclcpp_lifecycle::State & state){
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result =
			rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
	switch (state.id()) {
		case lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE:
			result = this->on_deactivate(get_current_state());
			if (result != rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS) {
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
MotionTracker::on_activate(const rclcpp_lifecycle::State & state){
	marker_array_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("markers",
				qos, callback, rclcpp::SubscriptionOptions(), msg_strategy);
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_deactivate(const rclcpp_lifecycle::State & state){
	marker_array_subscriber_.reset();
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_error(const rclcpp_lifecycle::State & state){
	RCLCPP_INFO(get_logger(), "An error occured");
	return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

int main(int argc, char const *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<MotionTracker>();
	executor.add_node(node->get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();

    return 0;
}

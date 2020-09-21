#ifndef INCLUDE_MOTIONTRACKER_HPP_
#define INCLUDE_MOTIONTRACKER_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class MotionTracker: public rclcpp_lifecycle::LifecycleNode {
public:
	MotionTracker();

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
	rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_subscriber_;
	rclcpp::QoS qos;
	rclcpp::message_memory_strategy::MessageMemoryStrategy<visualization_msgs::msg::MarkerArray>::SharedPtr msg_strategy;
	std::function<void(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)> callback;
	void markersReceivedCallback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);
};

#endif

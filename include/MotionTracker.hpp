#ifndef INCLUDE_MOTIONTRACKER_HPP_
#define INCLUDE_MOTIONTRACKER_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/point.hpp"

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
	rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr reference_joint_state_publisher_;
	rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr active_axis_changed_publisher_;
	rclcpp::QoS qos;
	rclcpp::message_memory_strategy::MessageMemoryStrategy<visualization_msgs::msg::MarkerArray>::SharedPtr msg_strategy;
	std::function<void(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)> callback;
	sensor_msgs::msg::JointState::SharedPtr reference_joint_state_;
	std_msgs::msg::Int8::SharedPtr active_joint_msg_;
	std::vector<double> lower_limits_rad_;
	std::vector<double> upper_limits_rad_;
	std::vector<double> segment_lengths_;
	void markersReceivedCallback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);
	static double distBetweenPoses(geometry_msgs::msg::Point& first, geometry_msgs::msg::Point& second);
};

#endif

#ifndef INCLUDE_MOTIONTRACKER_HPP_
#define INCLUDE_MOTIONTRACKER_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lifecycle_msgs/msg/state.hpp"

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
	rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int8>::SharedPtr active_axis_changed_publisher_;
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
	rcl_interfaces::msg::SetParametersResult onParamChange(const std::vector<rclcpp::Parameter> &parameters);
	bool canSetParameter(const rclcpp::Parameter &param);
	bool onLowerLimitsChangeRequest(const rclcpp::Parameter& param);
	bool onUpperLimitsChangeRequest(const rclcpp::Parameter& param);
	std::map<std::string, struct ParameterSetAccessRights> parameter_set_access_rights_;
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn SUCCESS =
	    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
};

}

#endif

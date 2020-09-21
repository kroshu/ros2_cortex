#include <memory>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iterator>

#include "MotionTracker.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"

MotionTracker::MotionTracker(): rclcpp_lifecycle::LifecycleNode("motion_tracker"), qos(rclcpp::QoS(rclcpp::KeepLast(1))),
	lower_limits_rad_(7), upper_limits_rad_(7), segment_lengths_({0.1575,0.2025,0.2045,0.2155,0.1895,0.2155,0.081})
{
	qos.best_effort();
	msg_strategy = std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<visualization_msgs::msg::MarkerArray>>();
	callback = [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) -> void
				{markersReceivedCallback(msg);};
}

double MotionTracker::distBetweenPoses(geometry_msgs::msg::Point& first, geometry_msgs::msg::Point& second){
	return sqrt(pow(std::fabs(first.x-second.x),2)+pow(std::fabs(first.y-second.y),2)+pow(std::fabs(first.z-second.z),2));
}

void MotionTracker::markersReceivedCallback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg){
//	for(auto it = msg->markers.begin(); it != msg->markers.end(); ++it){
//		RCLCPP_INFO(get_logger(), "Namespace: " + it->ns);
//		RCLCPP_INFO(get_logger(), "ID: " + std::to_string(it->id));
//
//		RCLCPP_INFO(get_logger(), "Point: x: " + std::to_string(it->pose.position.x) +
//				" y: " + std::to_string(it->pose.position.y) +
//				" z: " + std::to_string(it->pose.position.z));
//	}

	std::vector<visualization_msgs::msg::Marker> joint_markers;
	std::copy_if(msg->markers.begin(), msg->markers.end(), std::back_inserter(joint_markers),
			[](visualization_msgs::msg::Marker marker)->bool{return marker.ns == "joint_markers";});

	for(int active_joint=0; active_joint < 7; ++active_joint){
		active_joint_msg_->data = active_joint+1;
		active_axis_changed_publisher_->publish(*active_joint_msg_);
//		double distance = distBetweenPoses(joint_markers[active_joint+1].pose.position, joint_markers[active_joint].pose.position);
		double distance = 0.1575;
		int ulp = 5;
		if(std::fabs(segment_lengths_[active_joint]-distance) <= std::numeric_limits<double>::epsilon() * std::fabs(segment_lengths_[active_joint]+-distance)*ulp)
			RCLCPP_ERROR(get_logger(), "Markers impossible to track with robot");
		else{
//			TODO
//			Calculate rad position of joint based on the two endpositions (or it could be more than two)
//			e.g. we calculate the relative position of the moving endpoint to the constant base endpoint
//			- compare it to the relative pos at default state (probably pos = 0)
			float calculated_pos = 0;
			if(lower_limits_rad_[active_joint]*0.9 < calculated_pos && calculated_pos < upper_limits_rad_[active_joint]*0.9)
			{
				reference_joint_state_->position[active_joint] = calculated_pos;
				reference_joint_state_publisher_->publish(*reference_joint_state_);
			}
			else
			{
				RCLCPP_WARN(get_logger(), "Joint limit reached!");
			}
		}
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
	active_axis_changed_publisher_ = this->create_publisher<std_msgs::msg::Int8>("active_axis_changed", qos);
	active_joint_msg_ = std::make_shared<std_msgs::msg::Int8>();
	reference_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("reference_joint_state", qos);
	reference_joint_state_ = std::make_shared<sensor_msgs::msg::JointState>();
	reference_joint_state_->position.resize(7);
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

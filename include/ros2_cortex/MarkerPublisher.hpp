#ifndef INCLUDE_MARKERPUBLISHER_HPP_
#define INCLUDE_MARKERPUBLISHER_HPP_

#include "ros2_cortex/CortexClient.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ros2_cortex{

class MarkerPublisher: public CortexClient{
public:
     MarkerPublisher();
     void dataHandlerFunc_(sFrameOfData* fod);
     void errorMsgHandlerFunc_(int i_level, char* error_msg);
     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	 on_activate(const rclcpp_lifecycle::State & state);

	 rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
	 on_deactivate(const rclcpp_lifecycle::State & state);
private:
     rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
     visualization_msgs::msg::MarkerArray marker_array_;
};

}

#endif

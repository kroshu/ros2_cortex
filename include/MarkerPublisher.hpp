#ifndef INCLUDE_MARKERPUBLISHER_HPP_
#define INCLUDE_MARKERPUBLISHER_HPP_

#include "CortexClient.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class MarkerPublisher: public CortexClient{
public:
     explicit MarkerPublisher(const std::string& capture_file_name);
     void dataHandlerFunc_(sFrameOfData* fod);
     void errorMsgHandlerFunc_(int i_level, char* error_msg);
private:
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher_;
	visualization_msgs::msg::MarkerArray::SharedPtr marker_array_;
	static const std::vector<std::string> verb_levels;
};



#endif

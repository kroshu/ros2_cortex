#include <memory>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iterator>

#include "ros2_cortex/MotionTracker.hpp"
#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_cortex{

double d2r(double degrees)
{
  return degrees / 180 * M_PI;
}

MotionTracker::MotionTracker(): rclcpp_lifecycle::LifecycleNode("motion_tracker"), qos(rclcpp::QoS(rclcpp::KeepLast(1))),
	lower_limits_rad_(7), upper_limits_rad_(7), segment_lengths_({0.1575,0.2025,0.2045,0.2155,0.1895,0.2155,0.081})
{
	qos.best_effort();
	msg_strategy = std::make_shared<rclcpp::message_memory_strategy::MessageMemoryStrategy<visualization_msgs::msg::MarkerArray>>();
	callback = [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) -> void
				{markersReceivedCallback(msg);};
	this->declare_parameter("lower_limits_deg", rclcpp::ParameterValue(std::vector<double>( {-170, -120, -170, -120, -170,
	                                                                                           -120, -175})));
	this->declare_parameter("upper_limits_deg", rclcpp::ParameterValue(std::vector<double>( {170, 120, 170, 120, 170, 120,
																						   175})));

	this->set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter> &parameters)
	{ return this->onParamChange(parameters);});
	parameter_set_access_rights_.emplace("lower_limits_deg", ParameterSetAccessRights {true, true, true, false});
	parameter_set_access_rights_.emplace("upper_limits_deg", ParameterSetAccessRights {true, true, true, false});
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

	for(int active_joint=0; active_joint < 1; ++active_joint){
		active_joint_msg_->data = active_joint+1;
		active_axis_changed_publisher_->publish(*active_joint_msg_);
//		double distance = distBetweenPoses(joint_markers[active_joint+1].pose.position, joint_markers[active_joint].pose.position);
		double distance = 0.1575;
		int ulp = 5;
		double eps = std::numeric_limits<double>::epsilon() * std::fabs(segment_lengths_[active_joint]+distance)*ulp;
		if(std::fabs(segment_lengths_[active_joint]-distance) > eps)
			RCLCPP_ERROR(get_logger(), "Markers impossible to track with robot");
		else{
//			TODO
//			Calculate rad position of joint based on the two endpositions (or it could be more than two)
//			e.g. we calculate the relative position of the moving endpoint to the constant base endpoint
//			- compare it to the relative pos at default state (probably pos = 0)
			float calculated_pos = d2r(60);
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
	onLowerLimitsChangeRequest(this->get_parameter("lower_limits_deg"));
	onUpperLimitsChangeRequest(this->get_parameter("upper_limits_deg"));
	return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_cleanup(const rclcpp_lifecycle::State & state){
	reference_joint_state_->position.assign(7, 0);
	active_joint_msg_->data = 1;
	return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_shutdown(const rclcpp_lifecycle::State & state){
	rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn result =
			SUCCESS;
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
MotionTracker::on_activate(const rclcpp_lifecycle::State & state){
	marker_array_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>("markers",
				qos, callback, rclcpp::SubscriptionOptions(), msg_strategy);
	active_axis_changed_publisher_ = this->create_publisher<std_msgs::msg::Int8>("active_axis_changed", qos);
	active_joint_msg_ = std::make_shared<std_msgs::msg::Int8>();
	reference_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("reference_joint_state", qos);
	reference_joint_state_ = std::make_shared<sensor_msgs::msg::JointState>();
	reference_joint_state_->position.resize(7);
	reference_joint_state_publisher_->on_activate();
	active_axis_changed_publisher_->on_activate();
	return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_deactivate(const rclcpp_lifecycle::State & state){
	marker_array_subscriber_.reset();
	reference_joint_state_publisher_->on_deactivate();
	active_axis_changed_publisher_->on_deactivate();
	return SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_error(const rclcpp_lifecycle::State & state){
	RCLCPP_INFO(get_logger(), "An error occured");
	return SUCCESS;
}

rcl_interfaces::msg::SetParametersResult MotionTracker::onParamChange(
    const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  for (const rclcpp::Parameter &param : parameters)
  {
    if (param.get_name() == "lower_limits_deg" && canSetParameter(param))
    {
      result.successful = onLowerLimitsChangeRequest(param);
    }
    else if (param.get_name() == "upper_limits_deg" && canSetParameter(param))
    {
      result.successful = onUpperLimitsChangeRequest(param);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid parameter name %s", param.get_name().c_str());
    }
  }
  return result;
}

bool MotionTracker::canSetParameter(const rclcpp::Parameter &param)
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

bool MotionTracker::onLowerLimitsChangeRequest(const rclcpp::Parameter& param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
   {
     RCLCPP_ERROR(this->get_logger(), "Invalid parameter type for parameter %s",
                  param.get_name().c_str());
     return false;
   }


   if (param.as_double_array().size() != 7)
   {
     RCLCPP_ERROR(this->get_logger(), "Invalid parameter array length for parameter %s",
                  param.get_name().c_str());
     return false;
   }
   std::transform(param.as_double_array().begin(), param.as_double_array().end(), lower_limits_rad_.begin(), d2r);
   return true;
}

bool MotionTracker::onUpperLimitsChangeRequest(const rclcpp::Parameter& param)
{
  if (param.get_type() != rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY)
   {
     RCLCPP_ERROR(this->get_logger(), "Invalid parameter type for parameter %s",
                  param.get_name().c_str());
     return false;
   }
   if (param.as_double_array().size() != 7)
   {
     RCLCPP_ERROR(this->get_logger(), "Invalid parameter array length for parameter %s",
                  param.get_name().c_str());
     return false;
   }
   std::transform(param.as_double_array().begin(), param.as_double_array().end(), upper_limits_rad_.begin(), d2r);
   return true;
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

}
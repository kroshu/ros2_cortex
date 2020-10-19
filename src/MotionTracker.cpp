// Copyright 2020 Gergely Kovács
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iterator>
#include <vector>

#include "rclcpp/message_memory_strategy.hpp"
#include "rclcpp/rclcpp.hpp"
#include "kroshu_ros2_core/ROS2BaseNode.hpp"
#include "kroshu_ros2_core/Parameter.hpp"
#include "ros2_cortex/MotionTracker.hpp"

namespace ros2_cortex
{

double d2r(double degrees)
{
  return degrees / 180 * M_PI;
}

MotionTracker::MotionTracker()
: kroshu_ros2_core::ROS2BaseNode("motion_tracker"),
  lower_limits_rad_(joint_num_), upper_limits_rad_(joint_num_),
  original_joint_points_(joint_num_)
{
  original_joint_points_[0].x = 0.0;
  original_joint_points_[0].y = 0.0;
  original_joint_points_[0].z = segment_lengths_[0];
  for (int i = 1; i < joint_num_; ++i) {
    original_joint_points_[i].x = original_joint_points_[i - 1].x;
    original_joint_points_[i].y = original_joint_points_[i - 1].y;
    original_joint_points_[i].z = original_joint_points_[i - 1].z + segment_lengths_[i];
  }
  qos.best_effort();
  msg_strategy =
    std::make_shared<rclcpp::message_memory_strategy::
      MessageMemoryStrategy<visualization_msgs::msg::MarkerArray>>();
  callback = [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) -> void
    {markersReceivedCallback(msg);};
  marker_array_subscriber_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
    "markers",
    qos, callback, rclcpp::SubscriptionOptions(), msg_strategy);
  active_axis_changed_publisher_ = this->create_publisher<std_msgs::msg::Int8>(
    "active_axis_changed", qos);
  active_joint_msg_ = std::make_shared<std_msgs::msg::Int8>();
  active_joint_msg_->data = 1;
  reference_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "reference_joint_state", qos);
  reference_joint_state_ = std::make_shared<sensor_msgs::msg::JointState>();
  reference_joint_state_->position.resize(joint_num_);

  lower_limits_rad_ = {-170, -120, -170, -120, -170, -120, -175};
  upper_limits_rad_ = {170, 120, 170, 120, 170, 120, 175};
  kroshu_ros2_core::ROS2BaseNode::declareParameter("lower_limits_deg", rclcpp::ParameterValue(
      lower_limits_rad_),
    rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY, kroshu_ros2_core::ParameterSetAccessRights {
      true, true, true, false},
    std::bind(&MotionTracker::onLowerLimitsChangeRequest, this, std::placeholders::_1));
  kroshu_ros2_core::ROS2BaseNode::declareParameter("upper_limits_deg", rclcpp::ParameterValue(
      upper_limits_rad_),
    rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY, kroshu_ros2_core::ParameterSetAccessRights {
      true, true, true, false},
    std::bind(&MotionTracker::onUpperLimitsChangeRequest, this, std::placeholders::_1));

  this->set_on_parameters_set_callback([this](const std::vector<rclcpp::Parameter> & parameters)
    {return MotionTracker::onParamChange(parameters);});
}

double MotionTracker::distBetweenPoints(
  geometry_msgs::msg::Point & first,
  geometry_msgs::msg::Point & second)
{
  return sqrt(pow(std::fabs(first.x - second.x),
           2) + pow(std::fabs(first.y - second.y), 2) + pow(std::fabs(first.z - second.z), 2));
}

void MotionTracker::markersReceivedCallback(
  visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)
{
  std::vector<visualization_msgs::msg::Marker> joint_markers;
  std::copy_if(msg->markers.begin(), msg->markers.end(), std::back_inserter(joint_markers),
    [](const visualization_msgs::msg::Marker & marker) -> bool {
      return marker.ns == "joint_markers";
    });

  // Trying out changing joint 3
  for (int active_joint = 3; active_joint < 4; ++active_joint) {
    if (active_joint_msg_->data != active_joint + 1) {
      active_joint_msg_->data = static_cast<signed char>(active_joint + 1);
      active_axis_changed_publisher_->publish(*active_joint_msg_);
    }
    // double distance = distBetweenPoints(joint_markers[active_joint+1].pose.position,
    // joint_markers[active_joint].pose.position);
    double distance = segment_lengths_[active_joint];
    int ulp = 5;
    double eps = std::numeric_limits<double>::epsilon() *
      std::fabs(segment_lengths_[active_joint + 1] + distance) * ulp;
    if (std::fabs(segment_lengths_[active_joint] - distance) > eps) {
      RCLCPP_ERROR(get_logger(), "Markers impossible to track with robot");
    } else {
      // TODO(Gergely Kovacs)
      // Calculate rad position of joint based on the
      // two endpositions (or it could be more than two)
      // e.g. we calculate the relative position of the
      // moving endpoint to the constant base endpoint
      // - compare it to the relative pos at default state (probably pos = 0)
      // float distance_from_orig = distBetweenPoints(
      // joint_markers[active_joint+1].pose.position, original_joint_points_[active_joint+1]);
      // float calculated_pos = acos(1.0 - (pow(distance_from_orig,2)/
      // (2.0 * segment_lengths_[active_joint+1])));
      double calculated_pos = d2r(60);
      if (lower_limits_rad_[active_joint] * limit_eps_ < calculated_pos &&
        calculated_pos < upper_limits_rad_[active_joint] * limit_eps_)
      {
        auto current_time_ns = rclcpp_lifecycle::LifecycleNode::now().nanoseconds();
        reference_joint_state_->header.stamp.sec = static_cast<int>(current_time_ns / nss_in_s);
        reference_joint_state_->header.stamp.nanosec = static_cast<unsigned int>(current_time_ns -
          reference_joint_state_->header.stamp.sec * nss_in_s);
        reference_joint_state_->position[active_joint] = calculated_pos;
        reference_joint_state_publisher_->publish(*reference_joint_state_);
      } else {
        RCLCPP_WARN(get_logger(), "Joint limit reached!");
      }
    }
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_cleanup(const rclcpp_lifecycle::State & state)
{
  reference_joint_state_->position.assign(joint_num_, 0);
  active_joint_msg_->data = 1;
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_activate(const rclcpp_lifecycle::State & state)
{
  reference_joint_state_publisher_->on_activate();
  active_axis_changed_publisher_->on_activate();
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MotionTracker::on_deactivate(const rclcpp_lifecycle::State & state)
{
  marker_array_subscriber_.reset();
  reference_joint_state_publisher_->on_deactivate();
  active_axis_changed_publisher_->on_deactivate();
  return kroshu_ros2_core::ROS2BaseNode::SUCCESS;
}

bool MotionTracker::onLowerLimitsChangeRequest(const kroshu_ros2_core::Parameter & param)
{
  auto value = param.getValue().get<std::vector<double>>();
  if (value.size() != joint_num_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid parameter array length for parameter %s",
      param.getName().c_str());
    return false;
  }
  std::transform(value.begin(),
    value.end(), lower_limits_rad_.begin(), d2r);
  return true;
}

bool MotionTracker::onUpperLimitsChangeRequest(const kroshu_ros2_core::Parameter & param)
{
  auto value = param.getValue().get<std::vector<double>>();
  if (value.size() != joint_num_) {
    RCLCPP_ERROR(this->get_logger(), "Invalid parameter array length for parameter %s",
      param.getName().c_str());
    return false;
  }
  std::transform(value.begin(),
    value.end(), upper_limits_rad_.begin(), d2r);
  return true;
}

}  // namespace ros2_cortex

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ros2_cortex::MotionTracker>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}

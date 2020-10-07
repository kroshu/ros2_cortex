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

#ifndef ROS2_CORTEX__MOTIONTRACKER_HPP_
#define ROS2_CORTEX__MOTIONTRACKER_HPP_

#include <vector>
#include <string>
#include <map>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int8.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "ros2_cortex/ROS2BaseNode.hpp"

namespace ros2_cortex
{

class MotionTracker : public ROS2BaseNode
{
public:
  MotionTracker();

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

private:
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_subscriber_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr
    reference_joint_state_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int8>::SharedPtr
    active_axis_changed_publisher_;
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
  rclcpp::message_memory_strategy::MessageMemoryStrategy<visualization_msgs::msg::MarkerArray>::
  SharedPtr msg_strategy;
  std::function<void(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg)> callback;
  sensor_msgs::msg::JointState::SharedPtr reference_joint_state_;
  std_msgs::msg::Int8::SharedPtr active_joint_msg_;
  int joint_num_ = 7;
  std::vector<double> lower_limits_rad_;
  std::vector<double> upper_limits_rad_;
  std::vector<double> segment_lengths_ = {0.1575, 0.2025,
    0.2045, 0.2155,
    0.1895, 0.2155,
    0.081};
  std::vector<geometry_msgs::msg::Point> original_joint_points_;
  float limit_eps_ = 0.9;
  int nss_in_s = 1000000000;
  void markersReceivedCallback(visualization_msgs::msg::MarkerArray::ConstSharedPtr msg);
  static double distBetweenPoints(
    geometry_msgs::msg::Point & first,
    geometry_msgs::msg::Point & second);
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & parameters);
  bool onLowerLimitsChangeRequest(const rclcpp::Parameter & param);
  bool onUpperLimitsChangeRequest(const rclcpp::Parameter & param);
};

}  // namespace ros2_cortex

#endif  // ROS2_CORTEX__MOTIONTRACKER_HPP_

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

#ifndef ROS2_CORTEX__MARKERPUBLISHER_HPP_
#define ROS2_CORTEX__MARKERPUBLISHER_HPP_

#include <string>
#include "ros2_cortex/CortexClient.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ros2_cortex
{

class MarkerPublisher : public CortexClient
{
public:
  MarkerPublisher();
  void dataHandlerFunc_(sFrameOfData * fod);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

private:
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_array_publisher_;
  visualization_msgs::msg::MarkerArray marker_array_;
  int nss_in_s = 1000000000;
  std::string fps_comm = "GetContextFrameRate";
  rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(1));
};

}  // namespace ros2_cortex

#endif  // ROS2_CORTEX__MARKERPUBLISHER_HPP_

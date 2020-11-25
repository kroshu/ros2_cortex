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

#include <string>
#include <memory>
#include <functional>

#include "ros2_cortex/MarkerPublisher.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_cortex
{

MarkerPublisher::MarkerPublisher()
: CortexClientNode("marker_publisher")
{
  using namespace std::placeholders;
  cortex_client_->setDataHandlerFunc(
    [this](sFrameOfData & fod) {
      MarkerPublisher::dataHandlerFunc_(fod);
    });
  qos.best_effort();
  marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "markers",
    qos);
}

void MarkerPublisher::dataHandlerFunc_(sFrameOfData & frame_of_data)
{
  marker_array_.markers.clear();

  int n_ui_markers = frame_of_data.nUnidentifiedMarkers;
  for (int i = 0; i < n_ui_markers; ++i) {
    visualization_msgs::msg::Marker ui_marker;
    auto current_time_ns = rclcpp_lifecycle::LifecycleNode::now().nanoseconds();
    ui_marker.header.stamp.sec = static_cast<int>(current_time_ns / nss_in_s);
    ui_marker.header.stamp.nanosec =
      static_cast<unsigned int>(current_time_ns - ui_marker.header.stamp.sec * nss_in_s);
    ui_marker.ns = "ui_markers";
    ui_marker.id = i;
    ui_marker.action = 0;
    ui_marker.pose.position.x = frame_of_data.UnidentifiedMarkers[i][0];
    ui_marker.pose.position.y = frame_of_data.UnidentifiedMarkers[i][1];
    ui_marker.pose.position.z = frame_of_data.UnidentifiedMarkers[i][2];
    ui_marker.scale.x = 1;
    ui_marker.scale.y = 1;
    ui_marker.scale.z = 1;

    marker_array_.markers.emplace_back(ui_marker);
  }

  int n_bodies = frame_of_data.nBodies;
  for (int i_body = 0; i_body < n_bodies; ++i_body) {
    int n_markers = frame_of_data.BodyData[i_body].nMarkers;
    for (int i_marker = 0; i_marker < n_markers; ++i_marker) {
      visualization_msgs::msg::Marker marker;
      auto current_time_ns = rclcpp_lifecycle::LifecycleNode::now().nanoseconds();
      marker.header.stamp.sec = static_cast<int>(current_time_ns / nss_in_s);
      marker.header.stamp.nanosec =
        static_cast<unsigned int>(current_time_ns - marker.header.stamp.sec * nss_in_s);
      marker.ns = frame_of_data.BodyData[i_body].szName;
      marker.id = i_marker;
      marker.action = 0;
      marker.pose.position.x = frame_of_data.BodyData[i_body].Markers[i_marker][0];
      marker.pose.position.y = frame_of_data.BodyData[i_body].Markers[i_marker][1];
      marker.pose.position.z = frame_of_data.BodyData[i_body].Markers[i_marker][2];
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 1;

      marker_array_.markers.emplace_back(marker);
    }
  }

  marker_array_publisher_->publish(marker_array_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MarkerPublisher::on_activate(const rclcpp_lifecycle::State & state)
{
  marker_array_publisher_->on_activate();
  return CortexClientNode::on_activate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MarkerPublisher::on_deactivate(const rclcpp_lifecycle::State & state)
{
  marker_array_publisher_->on_deactivate();
  return CortexClientNode::on_deactivate(state);
}

}  // namespace ros2_cortex

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<ros2_cortex::MarkerPublisher>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();

  return 0;
}

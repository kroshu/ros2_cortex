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

#ifndef ROS2_CORTEX__CORTEXCLIENT_HPP_
#define ROS2_CORTEX__CORTEXCLIENT_HPP_

#include <condition_variable>
#include <functional>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include "lifecycle_msgs/msg/state.hpp"

#include "ros2_cortex/CortexMock.hpp"
#include "ros2_cortex/ROS2BaseNode.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_cortex
{

class CortexClient : public ROS2BaseNode
{
public:
  explicit CortexClient(const std::string & node_name);
  virtual ~CortexClient();
  virtual void dataHandlerFunc_(sFrameOfData * p_frame_of_data);
  virtual void errorMsgHandlerFunc_(int i_log_level, char * sz_log_message);
  int setDataHandlerFunc(void (* dataHandlerFunc)(sFrameOfData * p_frame_of_data));
  int setErrorMsgHandlerFunc(void (* errorMsgHandlerFunc)(int i_log_level, char * sz_log_message));
  int copyFrame(const sFrameOfData * p_src, sFrameOfData * p_dst) const;

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

private:
  std::string server_addr_ = "127.0.0.1";
  std::thread run_thread;
  void run();
  void exit();
  bool onCapFileNameChangeRequest(const rclcpp::Parameter & param);
  bool onRequestCommandChanged(const rclcpp::Parameter & param);
  std::string capture_file_path_ =
    "/home/rosdeveloper/ros2_ws/src/ros2_cortex/CaptureWithPlots1.json";
  bool cap_file_path_changed = false;

protected:
  CortexMock cortex_mock_;
  sFrameOfData current_fod_;
  template<typename T>
  struct Callback;
  typedef void (* data_callback_t)(sFrameOfData *);
  typedef void (* error_msg__callback_t)(int i_level, char * sz_msg);
};
}  // namespace ros2_cortex

#endif  // ROS2_CORTEX__CORTEXCLIENT_HPP_

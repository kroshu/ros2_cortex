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

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "ros2_cortex/CortexClient.hpp"
#include "kroshu_ros2_core/Parameter.hpp"
#include "kroshu_ros2_core/ROS2BaseNode.hpp"

namespace ros2_cortex
{

class CortexClientNode : public kroshu_ros2_core::ROS2BaseNode
{
public:
  explicit CortexClientNode(const std::string & node_name);
  virtual ~CortexClientNode();
  virtual void dataHandlerFunc_(sFrameOfData& frame_of_data);
  virtual void errorMsgHandlerFunc_(CortexVerbosityLevel log_level, const std::string& log_message);
  CortexReturn setDataHandlerFunc(std::function<void(sFrameOfData&)> dataHandlerFunc);
  CortexReturn setErrorMsgHandlerFunc(std::function<void(CortexVerbosityLevel,
                            const std::string&)> errorMsgHandlerFunc);
  CortexReturn copyFrame(const sFrameOfData& src, sFrameOfData& p_dst) const;


  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state);

  virtual rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state);

private:
  std::thread run_thread;
  void run();
  void exit();
  bool onCapFileNameChangeRequest(const kroshu_ros2_core::Parameter & param);
  bool onRequestCommandChanged(const kroshu_ros2_core::Parameter & param);
  void setHandlerFuncs();
  std::string capture_file_path_ =
    "/home/rosdeveloper/ros2_ws/src/ros2_cortex/CaptureWithPlots1.json";
  bool cap_file_path_changed = false;

protected:
  CortexClient cortex_client_;
  sFrameOfData current_fod_;
  template<typename T>
  struct Callback;
  typedef void (* data_callback_t)(sFrameOfData *);
  typedef void (* error_msg__callback_t)(int i_level, char * sz_msg);
};
}  // namespace ros2_cortex

#endif  // ROS2_CORTEX__CORTEXCLIENT_HPP_

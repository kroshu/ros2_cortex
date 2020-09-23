#include "ros2_cortex/MarkerPublisher.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_cortex{

template <typename T>
struct Callback;

template <typename Ret, typename... Params>
struct Callback<Ret(Params...)> {
   template <typename... Args>
   static Ret callback(Args... args) {
      return func(args...);
   }
   static std::function<Ret(Params...)> func;
};

template <typename Ret, typename... Params>
std::function<Ret(Params...)> Callback<Ret(Params...)>::func;

typedef void (*data_callback_t)(sFrameOfData*);
typedef void (*error_msg__callback_t)(int i_level, char *sz_msg);

MarkerPublisher::MarkerPublisher():CortexClient("marker_publisher"){
	Callback<void(sFrameOfData*)>::func = std::bind(&MarkerPublisher::dataHandlerFunc_, this, std::placeholders::_1);
	data_callback_t data_func = static_cast<data_callback_t>(Callback<void(sFrameOfData*)>::callback);
	setDataHandlerFunc(data_func);

	Callback<void(int i_level, char *sz_msg)>::func = std::bind(&MarkerPublisher::errorMsgHandlerFunc_, this, std::placeholders::_1, std::placeholders::_2);
	error_msg__callback_t error_msg_func = static_cast<error_msg__callback_t>(Callback<void(int i_level, char *sz_msg)>::callback);
	setErrorMsgHandlerFunc(error_msg_func);

	marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
}

void MarkerPublisher::dataHandlerFunc_(sFrameOfData* fod){
	marker_array_.markers.clear();

	int n_ui_markers = fod->nUnidentifiedMarkers;
	for(int i=0; i < n_ui_markers; ++i){
		visualization_msgs::msg::Marker ui_marker;
		ui_marker.header.stamp.sec = fod->TimeCode.iSeconds;
		ui_marker.header.stamp.nanosec = fod->TimeCode.iFrames * 5000000;
		ui_marker.ns = "ui_markers";
		ui_marker.id = i;
		ui_marker.action = 0;
		ui_marker.pose.position.x = fod->UnidentifiedMarkers[i][0];
		ui_marker.pose.position.y = fod->UnidentifiedMarkers[i][1];
		ui_marker.pose.position.z = fod->UnidentifiedMarkers[i][2];
		ui_marker.scale.x = 1;
		ui_marker.scale.y = 1;
		ui_marker.scale.z = 1;
		// TODO initialize other values

		marker_array_.markers.emplace_back(ui_marker);
	}

	int n_bodies = fod->nBodies;
	for(int i_body=0; i_body < n_bodies; ++i_body){
		int n_markers = fod->BodyData[i_body].nMarkers;
		for(int i_marker=0; i_marker < n_markers; ++i_marker){
			visualization_msgs::msg::Marker marker;
			marker.header.stamp.sec = fod->TimeCode.iSeconds;
			marker.header.stamp.nanosec = fod->TimeCode.iFrames * 5000000;
			marker.ns = fod->BodyData[i_body].szName;
			marker.id = i_marker;
			marker.action = 0;
			marker.pose.position.x = fod->BodyData[i_body].Markers[i_marker][0];
			marker.pose.position.y = fod->BodyData[i_body].Markers[i_marker][1];
			marker.pose.position.z = fod->BodyData[i_body].Markers[i_marker][2];
			marker.scale.x = 1;
			marker.scale.y = 1;
			marker.scale.z = 1;
			// TODO initialize other values

			marker_array_.markers.emplace_back(marker);
		}
	}

	marker_array_publisher_->publish(marker_array_);
}

void MarkerPublisher::errorMsgHandlerFunc_(int i_level, char* error_msg){
	switch(i_level){
		case 1:
			RCLCPP_ERROR(get_logger(), static_cast<std::string>(error_msg));
			break;
		case 2:
			RCLCPP_WARN(get_logger(), static_cast<std::string>(error_msg));
			break;
		case 3:
			RCLCPP_INFO(get_logger(), static_cast<std::string>(error_msg));
			break;
		case 4:
			RCLCPP_DEBUG(get_logger(), static_cast<std::string>(error_msg));
			break;
		default:
			break;
	}
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MarkerPublisher::on_activate(const rclcpp_lifecycle::State & state){
	marker_array_publisher_->on_activate();
	return CortexClient::on_activate(state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MarkerPublisher::on_deactivate(const rclcpp_lifecycle::State & state){
	marker_array_publisher_->on_deactivate();
	return CortexClient::on_deactivate(state);
}

}

int main(int argc, char const *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<ros2_cortex::MarkerPublisher>();
	executor.add_node(node->get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();

    return 0;
}

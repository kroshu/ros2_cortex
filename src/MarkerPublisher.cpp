#include "MarkerPublisher.hpp"
#include "rclcpp/rclcpp.hpp"

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

const std::vector<std::string> MarkerPublisher:: verb_levels ({"None", "Error", "Warning", "Info", "Debug"});

MarkerPublisher::MarkerPublisher(const std::string& capture_file_name):CortexClient(capture_file_name){
	Callback<void(sFrameOfData*)>::func = std::bind(&MarkerPublisher::dataHandlerFunc_, this, std::placeholders::_1);
	data_callback_t data_func = static_cast<data_callback_t>(Callback<void(sFrameOfData*)>::callback);
	setDataHandlerFunc(data_func);

	Callback<void(int i_level, char *sz_msg)>::func = std::bind(&MarkerPublisher::errorMsgHandlerFunc_, this, std::placeholders::_1, std::placeholders::_2);
	error_msg__callback_t error_msg_func = static_cast<error_msg__callback_t>(Callback<void(int i_level, char *sz_msg)>::callback);
	setErrorMsgHandlerFunc(error_msg_func);

	marker_array_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
}

void MarkerPublisher::dataHandlerFunc_(sFrameOfData* fod){
	marker_array_->markers.clear();

	int n_ui_markers = fod->nUnidentifiedMarkers;
	for(int i=0; i < n_ui_markers; ++i){
		visualization_msgs::msg::Marker::SharedPtr ui_marker;
		ui_marker->ns = "ui_markers";
		ui_marker->action = 0;
		ui_marker->pose.position.x = fod->UnidentifiedMarkers[i][0];
		ui_marker->pose.position.y = fod->UnidentifiedMarkers[i][1];
		ui_marker->pose.position.z = fod->UnidentifiedMarkers[i][2];
		// TODO initialize other values

		marker_array_->markers.emplace_back(*ui_marker);
	}

	int n_bodies = fod->nBodies;
	for(int i_body=0; i_body < n_bodies; ++i_body){
		int n_markers = fod->BodyData[i_body].nMarkers;
		for(int i_marker=0; i_marker < n_markers; ++i_marker){
			visualization_msgs::msg::Marker::SharedPtr marker;
			marker->ns = fod->BodyData[i_body].szName;
			marker->action = 0;
			marker->pose.position.x = fod->BodyData[i_body].Markers[i_marker][0];
			marker->pose.position.y = fod->BodyData[i_body].Markers[i_marker][1];
			marker->pose.position.z = fod->BodyData[i_body].Markers[i_marker][2];
			// TODO initialize other values

			marker_array_->markers.emplace_back(*marker);
		}
	}

	marker_array_publisher_->publish(*marker_array_);
}

void MarkerPublisher::errorMsgHandlerFunc_(int i_level, char* error_msg){
	RCLCPP_ERROR(get_logger(), i_level+": "+static_cast<std::string>(error_msg));
}

int main(int argc, char const *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto node = std::make_shared<MarkerPublisher>("CaptureWithPlots1.json");
	executor.add_node(node->get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();

    return 0;
}

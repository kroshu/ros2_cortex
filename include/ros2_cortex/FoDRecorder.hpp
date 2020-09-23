#ifndef FOD_RECORDER_HPP_
#define FOD_RECORDER_HPP_

#include <vector>

#include "Cortex.h"
#include "rapidjson/document.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/writer.h"

namespace ros2_cortex{

class FoDRecorder{
public:
    FoDRecorder(std::string talk_to_host_addr, std::string host_addr, const std::string& capture_file_name, int capture_size, int file_write_buffer_size = 65536);
private:
    const std::string capture_file_name_;
    rapidjson::Document json_doc_;
    const int capture_size_, file_write_buffer_size_;
    int frame_count = 0;
    static const std::vector<std::string> verb_levels;
    void printBodyDefs(sBodyDefs& body_defs);
    void printBodyDef(sBodyDef& body_def, rapidjson::Value& body_def_array_json, rapidjson::Document::AllocatorType& allocator);
    void printMarkerData(const std::vector<tMarkerData>& marker_data, rapidjson::Value& markers_json, rapidjson::Document::AllocatorType& allocator);
    void printSegmentData(const std::vector<tSegmentData>& segment_data, rapidjson::Value& segments_json, rapidjson::Document::AllocatorType& allocator);
    void printForceData(const std::vector<tForceData>& force_data, rapidjson::Value& forces_json, rapidjson::Document::AllocatorType& allocator);
    void printBodyDatas(const std::vector<sBodyData>& body_data, rapidjson::Value& bodies_json, rapidjson::Document::AllocatorType& allocator);
    void printAnalogData(const sAnalogData& analog_data, rapidjson::Value& ad_value, rapidjson::Document::AllocatorType& allocator);
    void printFrameOfData(const sFrameOfData& frame_of_data);
    void errorMsgPrinter(int i_level, char *sz_msg);
    void dataPrinter(sFrameOfData* frame_of_data);
};

}

#endif

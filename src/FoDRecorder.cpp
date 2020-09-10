#include <iostream>
#include <functional>
#include <array>
#include <algorithm>

#include "FoDRecorder.hpp"

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

FoDRecorder::FoDRecorder(const std::string& capture_file_name, int capture_size, int file_write_buffer_size)
:capture_file_name_(capture_file_name), capture_size_(capture_size), file_write_buffer_size_(file_write_buffer_size){
    json_doc_.SetObject();
    rapidjson::Value frames_array_json(rapidjson::kArrayType);
    json_doc_.AddMember("framesArray", frames_array_json, json_doc_.GetAllocator());

    Callback<void(sFrameOfData*)>::func = std::bind(&FoDRecorder::dataPrinter, this, std::placeholders::_1);
    data_callback_t data_func = static_cast<data_callback_t>(Callback<void(sFrameOfData*)>::callback);      
    Cortex_SetDataHandlerFunc(data_func);

    Callback<void(int i_level, char *sz_msg)>::func = std::bind(&FoDRecorder::errorMsgPrinter, this, std::placeholders::_1, std::placeholders::_2);
    error_msg__callback_t error_msg_func = static_cast<error_msg__callback_t>(Callback<void(int i_level, char *sz_msg)>::callback);      
    Cortex_SetErrorMsgHandlerFunc(error_msg_func);

    printBodyDefs(*Cortex_GetBodyDefs());
}

const std::vector<std::string> FoDRecorder:: verb_levels ({"None", "Error", "Warning", "Info", "Debug"});

void FoDRecorder::printBodyDefs(sBodyDefs& body_defs){
    rapidjson::Document::AllocatorType& allocator = json_doc_.GetAllocator();
    rapidjson::Value body_defs_json(rapidjson::kObjectType);
    json_doc_.AddMember("bodyDefs", body_defs_json, allocator);

    body_defs_json.AddMember("nBodyDefs", body_defs.nBodyDefs, allocator);
    int n_body_defs = body_defs.nBodyDefs;
    rapidjson::Value body_def_array_json(rapidjson::kArrayType);
	for (int i = 0; i < n_body_defs; i++)
	{
		printBodyDef(body_defs.BodyDefs[i], body_def_array_json, allocator);
	}
    body_defs_json.AddMember("bodyDefs", body_def_array_json, allocator);

    body_defs_json.AddMember("nAnalogChannels", body_defs.nAnalogChannels, allocator);
    int n_analog_channels = body_defs.nAnalogChannels;
	char** src_analogch_names_ptr = body_defs.szAnalogChannelNames;
    rapidjson::Value analogch_names_json(rapidjson::kArrayType);
	for (int i = 0; i < n_analog_channels; ++i, ++src_analogch_names_ptr)
	{
        analogch_names_json.PushBack(rapidjson::StringRef(*src_analogch_names_ptr), allocator);
	}
    body_defs_json.AddMember("analogChannelNames", analogch_names_json, allocator);

    body_defs_json.AddMember("nForcePlates", body_defs.nForcePlates, allocator);
    body_defs_json.AddMember("analogBitDepth", body_defs.AnalogBitDepth, allocator);

    rapidjson::Value analog_lo_voltage_json(rapidjson::kArrayType);
    rapidjson::Value analog_hi_voltage_json(rapidjson::kArrayType);
	for (int i = 0; i < n_analog_channels; i++)
	{
        analog_lo_voltage_json.PushBack(body_defs.AnalogLoVoltage[i], allocator);
        analog_hi_voltage_json.PushBack(body_defs.AnalogHiVoltage[i], allocator);
	}
}

void FoDRecorder::printBodyDef(sBodyDef& body_def, rapidjson::Value& body_def_array_json, rapidjson::Document::AllocatorType& allocator){
    rapidjson::Value body_def_json(rapidjson::kObjectType);
    body_def_json.AddMember("name", rapidjson::StringRef(body_def.szName), allocator);

    body_def_json.AddMember("nMarkers", body_def.nMarkers, allocator);
    int n_markers = body_def.nMarkers;
	char** src_marker_names_ptr = body_def.szMarkerNames;
    rapidjson::Value marker_names_json(rapidjson::kArrayType);
	for (int i = 0; i < n_markers; ++i, ++src_marker_names_ptr)
	{
        marker_names_json.PushBack(rapidjson::StringRef(*src_marker_names_ptr), allocator);
	}
    body_def_json.AddMember("markerNames", marker_names_json, allocator);

    rapidjson::Value hierarchy_json(rapidjson::kObjectType);
    body_def_json.AddMember("nSegments", body_def.Hierarchy.nSegments, allocator);
    int n_segments = body_def.Hierarchy.nSegments;
    char** src_segment_names_ptr = body_def.Hierarchy.szSegmentNames;
    rapidjson::Value segment_names_json(rapidjson::kArrayType);
    rapidjson::Value parents_json(rapidjson::kArrayType);
    for (int i = 0; i < n_segments; ++i, ++src_segment_names_ptr)
	{
        segment_names_json.PushBack(rapidjson::StringRef(*src_segment_names_ptr), allocator);
        parents_json.PushBack(body_def.Hierarchy.iParents[i], allocator);
    }
    hierarchy_json.AddMember("segmentNames", segment_names_json, allocator);
    hierarchy_json.AddMember("parents", parents_json, allocator);
    body_def_json.AddMember("hierarchy", hierarchy_json, allocator);

    body_def_json.AddMember("nDofs", body_def.nDofs, allocator);
    int n_dofs = body_def.nDofs;
	char** src_dof_names_ptr = body_def.szDofNames;
    rapidjson::Value dof_names_json(rapidjson::kArrayType);
	for (int i = 0; i < n_dofs; ++i, ++src_dof_names_ptr)
	{
        dof_names_json.PushBack(rapidjson::StringRef(*src_dof_names_ptr), allocator);
	}
    body_def_json.AddMember("dofNames", dof_names_json, allocator);
}

void FoDRecorder::errorMsgPrinter(int i_level, char *sz_msg)
{
    std::cerr << verb_levels[i_level] << ": " << sz_msg << std::endl;
}

void FoDRecorder::printMarkerData(const std::vector<tMarkerData>& marker_data, rapidjson::Value& markers_json, rapidjson::Document::AllocatorType& allocator){
    for(std::vector<tMarkerData>::const_iterator it = marker_data.begin(); it != marker_data.end(); ++it) {
        rapidjson::Value marker_json(rapidjson::kObjectType);
        marker_json.AddMember("x", (*it)[0], allocator);
        marker_json.AddMember("y", (*it)[1], allocator);
        marker_json.AddMember("z", (*it)[2], allocator);
        markers_json.PushBack(marker_json, allocator);
    }
}

void FoDRecorder::printSegmentData(const std::vector<tSegmentData>& segment_data, rapidjson::Value& segments_json, rapidjson::Document::AllocatorType& allocator){
    for(std::vector<tSegmentData>::const_iterator it = segment_data.begin(); it != segment_data.end(); ++it) {
        rapidjson::Value segment_json(rapidjson::kObjectType);
        segment_json.AddMember("x", (*it)[0], allocator);
        segment_json.AddMember("y", (*it)[1], allocator);
        segment_json.AddMember("z", (*it)[2], allocator);
        segment_json.AddMember("aX", (*it)[3], allocator);
        segment_json.AddMember("aY", (*it)[4], allocator);
        segment_json.AddMember("aZ", (*it)[5], allocator);
        segment_json.AddMember("length", (*it)[6], allocator);
        segments_json.PushBack(segment_json, allocator);
    }
}

void FoDRecorder::printForceData(const std::vector<tForceData>& force_data, rapidjson::Value& forces_json, rapidjson::Document::AllocatorType& allocator){
    for(std::vector<tForceData>::const_iterator it = force_data.begin(); it != force_data.end(); ++it) {
        rapidjson::Value force_json(rapidjson::kObjectType);
        force_json.AddMember("x", (*it)[0], allocator);
        force_json.AddMember("y", (*it)[1], allocator);
        force_json.AddMember("z", (*it)[2], allocator);
        force_json.AddMember("fX", (*it)[3], allocator);
        force_json.AddMember("fY", (*it)[4], allocator);
        force_json.AddMember("fZ", (*it)[5], allocator);
        force_json.AddMember("mZ", (*it)[6], allocator);
        forces_json.PushBack(force_json, allocator);
    }
}

void FoDRecorder::printBodyDatas(const std::vector<sBodyData>& body_data, rapidjson::Value& bodies_json, rapidjson::Document::AllocatorType& allocator){
    int i=0;
    for(std::vector<sBodyData>::const_iterator it = body_data.begin(); it != body_data.end(); ++it) {
        rapidjson::Value body_json(rapidjson::kObjectType);
        body_json.AddMember("name", rapidjson::StringRef(it->szName), allocator);

        int n_markers = it->nMarkers;
        body_json.AddMember("nMarkers",n_markers, allocator);
        rapidjson::Value markers_json(rapidjson::kArrayType);
        if(n_markers > 0){
            // TODO is this not fully unnecessary compared to just handing over the pointer to the function
            std::vector<tMarkerData> markers(n_markers);
            auto marker_ptr = it->Markers;
            for (int i = 0; i < n_markers; ++i, ++marker_ptr)
            {
                memcpy(markers[i],*marker_ptr, sizeof(tMarkerData));
            }
            printMarkerData(markers, markers_json, allocator);
        }
        body_json.AddMember("markers", markers_json, allocator);
        body_json.AddMember("fAvgMarkerResidual", it->fAvgMarkerResidual, allocator);

        int n_segments = it->nSegments;
        body_json.AddMember("nSegments", n_segments, allocator);
        rapidjson::Value segments_json(rapidjson::kArrayType);
        if(n_segments > 0){
            // TODO is this not fully unnecessary compared to just handing over the pointer to the function
            std::vector<tSegmentData> segments(n_markers);
            auto segment_ptr = it->Segments;
            for (int i = 0; i < n_segments; ++i, ++segment_ptr)
            {
                memcpy(segments[i],*segment_ptr, sizeof(tSegmentData));
            }
            printSegmentData(segments, segments_json, allocator);
        }
        body_json.AddMember("segments", segments_json, allocator);

        body_json.AddMember("nDofs", it->nDofs, allocator);
        rapidjson::Value dofs_json(rapidjson::kArrayType);
        for (i=0 ; i<it->nDofs ; i++)
        {
            dofs_json.PushBack(it->Dofs[i], allocator);
        }
        body_json.AddMember("dofs", dofs_json, allocator);
        body_json.AddMember("fAvgDofResidual", it->fAvgDofResidual, allocator);
        body_json.AddMember("nIterations", it->nIterations, allocator);

        body_json.AddMember("encoderZoom", it->ZoomEncoderValue, allocator);
        body_json.AddMember("encoderFocus", it->FocusEncoderValue, allocator);
        body_json.AddMember("encoderIris", it->IrisEncoderValue, allocator);
        rapidjson::Value cam_track_params_json(rapidjson::kObjectType);
        cam_track_params_json.AddMember("offsetX", it->CamTrackParams[0], allocator);
        cam_track_params_json.AddMember("offsetY", it->CamTrackParams[1], allocator);
        cam_track_params_json.AddMember("offsetZ", it->CamTrackParams[2], allocator);
        cam_track_params_json.AddMember("offsetAngleX", it->CamTrackParams[3], allocator);
        cam_track_params_json.AddMember("offsetAngleY", it->CamTrackParams[4], allocator);
        cam_track_params_json.AddMember("offsetAngleZ", it->CamTrackParams[5], allocator);
        cam_track_params_json.AddMember("videoWidth", it->CamTrackParams[6], allocator);
        cam_track_params_json.AddMember("videoHeight", it->CamTrackParams[7], allocator);
        cam_track_params_json.AddMember("opticalCenterX", it->CamTrackParams[8], allocator);
        cam_track_params_json.AddMember("opticalCenterY", it->CamTrackParams[9], allocator);
        cam_track_params_json.AddMember("fovX", it->CamTrackParams[10], allocator);
        cam_track_params_json.AddMember("fovY", it->CamTrackParams[11], allocator);
        cam_track_params_json.AddMember("pixelAspect", it->CamTrackParams[12], allocator);
        cam_track_params_json.AddMember("firstCoefficient", it->CamTrackParams[13], allocator);
        body_json.AddMember("camTrackParams", cam_track_params_json, allocator);

        body_json.AddMember("nEvents", it->nEvents, allocator);
        rapidjson::Value events_json(rapidjson::kArrayType);
        for (i=0 ; i<it->nEvents ; i++)
        {
            events_json.PushBack(rapidjson::StringRef(it->Events[i]), allocator);
        }
        body_json.AddMember("events", events_json, allocator);

        bodies_json.PushBack(body_json, allocator);
    }
    
}

void FoDRecorder::printAnalogData(const sAnalogData& analog_data, rapidjson::Value& ad_value, rapidjson::Document::AllocatorType& allocator){
    int n_analog_samples = analog_data.nAnalogSamples;
    int n_analog_channels = analog_data.nAnalogChannels;
    int n_analog = n_analog_samples * n_analog_channels;
    ad_value.AddMember("nAnalogChannels", n_analog_channels, allocator);
    ad_value.AddMember("nAnalogSamples", n_analog_samples, allocator);

    rapidjson::Value analog_samples_json(rapidjson::kArrayType);
    if(n_analog > 0){
        const std::vector<short> analog_samples(analog_data.AnalogSamples, analog_data.AnalogSamples + n_analog);
        for(std::vector<short>::const_iterator it = analog_samples.begin(); it != analog_samples.end(); ++it) {
            rapidjson::Value sample_json(rapidjson::kObjectType);
            sample_json.AddMember("value", *it, allocator);
            analog_samples_json.PushBack(sample_json, allocator);
        }
    }
    ad_value.AddMember("analogSamples", analog_samples_json, allocator);

    int n_force_plates = analog_data.nForcePlates;
    int n_force_samples = analog_data.nForceSamples;
    int n_forces = n_force_plates * n_force_samples;
    ad_value.AddMember("nForcePlates", n_force_plates, allocator);
    ad_value.AddMember("nForceSamples", n_force_samples, allocator);
    rapidjson::Value forces_json(rapidjson::kArrayType);
    if(n_forces > 0){
        // TODO is this not fully unnecessary compared to just handing over the pointer to the function
        std::vector<tForceData> forces(n_forces);
        auto force_ptr = analog_data.Forces;
        for (int i = 0; i < n_forces; ++i, ++force_ptr)
        {
            memcpy(forces[i],*force_ptr, sizeof(tForceData));
        }
        printForceData(forces, forces_json, allocator);
        ad_value.AddMember("forces", forces_json, allocator);
    }

    int n_angle_encoders = analog_data.nAngleEncoders;
	int n_angle_encoder_samples = analog_data.nAngleEncoderSamples;
    int n_all_ae__samples = n_angle_encoder_samples * n_angle_encoders;
    ad_value.AddMember("nAngleEncoders", n_angle_encoders, allocator);
    ad_value.AddMember("nAngleEncoderSamples", n_angle_encoder_samples, allocator);

    rapidjson::Value angle_encoder_samples_json(rapidjson::kArrayType);
    if(n_all_ae__samples > 0){
        const std::vector<double> angle_encoder_samples(analog_data.AngleEncoderSamples, analog_data.AngleEncoderSamples + n_all_ae__samples);
        for(std::vector<double>::const_iterator it = angle_encoder_samples.begin(); it != angle_encoder_samples.end(); ++it) {
            rapidjson::Value sample_json(rapidjson::kObjectType);
            sample_json.AddMember("value", *it, allocator);
            angle_encoder_samples_json.PushBack(sample_json, allocator);
        }
    }
    ad_value.AddMember("angleEncoderSamples", angle_encoder_samples_json, allocator);
}

void FoDRecorder::printFrameOfData(const sFrameOfData& frame_of_data)
{
    rapidjson::Document::AllocatorType& allocator = json_doc_.GetAllocator(); // TODO maybe instead get as param??
    rapidjson::Value frame_json(rapidjson::kObjectType);
    frame_json.AddMember("frame", frame_of_data.iFrame, allocator);
    frame_json.AddMember("frameDelay", frame_of_data.fDelay, allocator);
    
    int n_bodies = frame_of_data.nBodies;
    frame_json.AddMember("nBodies", n_bodies, allocator);
    rapidjson::Value bodies_json(rapidjson::kArrayType);
    if(n_bodies > 0){
        std::vector<sBodyData> body_datas(frame_of_data.BodyData, frame_of_data.BodyData + n_bodies);
        printBodyDatas(body_datas, bodies_json, allocator);
    }
    frame_json.AddMember("bodies", bodies_json, allocator);

    int n_ui_markers = frame_of_data.nUnidentifiedMarkers;
    frame_json.AddMember("nUnidentifiedMarkers", n_ui_markers, allocator);
    rapidjson::Value ui_markers_json(rapidjson::kArrayType);
    if(n_ui_markers > 0){
        // TODO is this not fully unnecessary compared to just handing over the pointer to the function
        std::vector<tMarkerData> ui_markers(n_ui_markers);
        auto ui_marker_ptr = frame_of_data.UnidentifiedMarkers;
        for (int i = 0; i < n_ui_markers; ++i, ++ui_marker_ptr)
        {
            memcpy(ui_markers[i],*ui_marker_ptr, sizeof(tMarkerData));
        }
        printMarkerData(ui_markers, ui_markers_json, allocator);
    }
    frame_json.AddMember("unidentifiedMarkers", ui_markers_json, allocator);

    rapidjson::Value analog_data_json(rapidjson::kObjectType);
    printAnalogData(frame_of_data.AnalogData, analog_data_json, allocator);
    frame_json.AddMember("analogData", analog_data_json, allocator);

    // TODO is this copy unnecessary to make it more readable
    sRecordingStatus rc = frame_of_data.RecordingStatus;
    rapidjson::Value rcstatus_json(rapidjson::kObjectType);
    rcstatus_json.AddMember("recording", rc.bRecording, allocator);
    rcstatus_json.AddMember("firstFrame", rc.iFirstFrame, allocator);
    rcstatus_json.AddMember("lastFrame", rc.iLastFrame, allocator);
    rcstatus_json.AddMember("captureFileName", rapidjson::StringRef(rc.szFilename), allocator);
    frame_json.AddMember("recordingStatus", rcstatus_json, allocator);

    rapidjson::Value timecode_json(rapidjson::kObjectType);
    sTimeCode tc = frame_of_data.TimeCode;
    timecode_json.AddMember("hours", tc.iHours, allocator);
    timecode_json.AddMember("minutes", tc.iMinutes, allocator);
    timecode_json.AddMember("seconds", tc.iSeconds, allocator);
    timecode_json.AddMember("frames", tc.iFrames, allocator);
    timecode_json.AddMember("standard", tc.iStandard, allocator);
    frame_json.AddMember("timeCode", timecode_json, allocator);
    
    json_doc_["framesArray"].PushBack(frame_json, allocator);
}

void FoDRecorder::dataPrinter(sFrameOfData* frame_of_data){
    if (frame_count >= capture_size_+1) return;
    if (frame_count >= capture_size_)
    {
        FILE* fp = fopen(capture_file_name_.data(), "wb");

        char write_buffer[file_write_buffer_size_];
        rapidjson::FileWriteStream os(fp, write_buffer, sizeof(write_buffer));

        rapidjson::Writer<rapidjson::FileWriteStream> writer(os);
        json_doc_.Accept(writer);

        fclose(fp);
        frame_count++;
        return;
    }

    printFrameOfData(*frame_of_data);
    frame_count++;
}

int main(int argc, char* argv[])
{
    FoDRecorder recorder("CaptureWithPlots1.json", 7230);
    std::cout << "Usage: ClientTest <Me> <Cortex>" << std::endl;
    std::cout << "       Me = My machine name or its IP address" << std::endl;
    std::cout << "       Cortex = Cortex's machine name or its IP Address" << std::endl;
    std::cout << "----------" << std::endl;


    sHostInfo cortex_hostInfo;
    int ret_val;
    unsigned char sdk_version[4];
    char key;

    Cortex_SetVerbosityLevel(VL_Info);
    ret_val = Cortex_GetSdkVersion(sdk_version);
    std::cout << "Using SDK Version: " << static_cast<int>(sdk_version[1]) <<
            "." << static_cast<int>(sdk_version[2]) <<
            "." << static_cast<int>(sdk_version[3]) << "." << std::endl;

    if (argc == 1)
    {
        ret_val = Cortex_Initialize((char*)"", (char*)NULL);
    }
    else
    if (argc == 2)
    {
        ret_val = Cortex_Initialize(argv[1], (char*)NULL);
    }
    else
    if (argc == 3)
    {
        ret_val = Cortex_Initialize(argv[1], argv[2]);
    }

    if (ret_val != RC_Okay)
    {
        std::cerr << "Error: Unable to initialize ethernet communication" << std::endl;
        return -1;
    }

    ret_val = Cortex_GetHostInfo(&cortex_hostInfo);

	if (ret_val != RC_Okay
     || !cortex_hostInfo.bFoundHost)
    {
        std::cerr << "Cortex not found." << std::endl;
        return -1;
    }
    else
    {
        std::cout << "Found " << cortex_hostInfo.szHostProgramName
                  << " Version " << static_cast<int>(cortex_hostInfo.HostProgramVersion[1])
                  << "." << static_cast<int>(cortex_hostInfo.HostProgramVersion[2])
                  << "." << static_cast<int>(cortex_hostInfo.HostProgramVersion[3])
                  << " at " << static_cast<int>(cortex_hostInfo.HostMachineAddress[0])
                  << "." <<static_cast<int>(cortex_hostInfo.HostMachineAddress[1])
                  << "." << static_cast<int>(cortex_hostInfo.HostMachineAddress[2])
                  << "." << static_cast<int>(cortex_hostInfo.HostMachineAddress[3])
                  << " (" << cortex_hostInfo.szHostMachineName << ")" << std::endl;
    }

    while (1)
    {
        key = getchar();
        if (key == 'Q'
         || key == 'q')
        {
            break;
        }
    }

    ret_val = Cortex_Exit();
    std::cout << "Press any key to continue...";
    key = getchar();

    return 0;
}
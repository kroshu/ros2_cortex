#include <iostream>
#include <functional>
#include <array>
#include <algorithm>

#include "FoDRecorder.hpp"

FoDRecorder::FoDRecorder(const std::string& capture_file_name, int capture_size, int file_write_buffer_size = 65536)
:capture_file_name_(capture_file_name), capture_size_(capture_size), file_write_buffer_size_(file_write_buffer_size){
    json_doc_.SetObject();
    rapidjson::Value framesArray(rapidjson::kArrayType);
    json_doc_.AddMember("framesArray", framesArray, json_doc_.GetAllocator());
}

void FoDRecorder::errorMsgPrinter(int i_level, char *sz_msg)
{
    std::cerr << verb_level[i_level] << ": " << sz_msg;
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
        // TODO check whether this is safe
        body_json.AddMember("name", rapidjson::StringRef(it->szName), allocator);

        int n_markers = it->nMarkers;
        body_json.AddMember("nMarkers",n_markers, allocator);
        rapidjson::Value markers_json(rapidjson::kArrayType);
        if(n_markers > 0){
            
            std::vector<tMarkerData> markers(it->Markers, it->Markers + n_markers);
            printMarkerData(markers, markers_json, allocator);
        }
        body_json.AddMember("markers", markers_json, allocator);
        body_json.AddMember("fAvgMarkerResidual", it->fAvgMarkerResidual, allocator);

        int n_segments = it->nSegments;
        body_json.AddMember("nSegments", n_segments, allocator);
        rapidjson::Value segments_json(rapidjson::kArrayType);
        if(n_segments > 0){
            std::vector<tSegmentData> segments(it->Segments, it->Segments + n_segments);
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
    if(n_analog_samples > 0 && n_analog_channels > 0){
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
    if(n_force_plates > 0 && n_force_samples > 0){
        std::vector<tForceData> forces(analog_data.Forces, analog_data.Forces + n_forces);
        printForceData(forces, forces_json, allocator);
        ad_value.AddMember("forces", forces_json, allocator);
    }

    int n_angle_encoders = analog_data.nAngleEncoders;
	int n_angle_encoder_samples = analog_data.nAngleEncoderSamples;
    int n_all_ae__samples = n_angle_encoder_samples * n_angle_encoders;
    ad_value.AddMember("nAngleEncoders", n_angle_encoders, allocator);
    ad_value.AddMember("nAngleEncoderSamples", n_angle_encoder_samples, allocator);

    rapidjson::Value angle_encoder_samples_json(rapidjson::kArrayType);
    if(n_angle_encoders > 0 && n_angle_encoder_samples > 0){
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

    int n_unidentified_markers = frame_of_data.nUnidentifiedMarkers;
    frame_json.AddMember("nUnidentifiedMarkers", n_unidentified_markers, allocator);
    rapidjson::Value ui_markers_json(rapidjson::kArrayType);
    if(n_unidentified_markers > 0){
        std::vector<tMarkerData> ui_markers(frame_of_data.UnidentifiedMarkers, frame_of_data.UnidentifiedMarkers + n_unidentified_markers);
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
    std::cout << "Usage: ClientTest <Me> <Cortex>\n";
    std::cout << "       Me = My machine name or its IP address\n";
    std::cout << "       Cortex = Cortex's machine name or its IP Address\n";
    std::cout << "----------\n";


    sHostInfo Cortex_HostInfo;
    int retval;
    unsigned char SDK_Version[4];
    char key;
    int i;

    Cortex_SetVerbosityLevel(VL_Info);
    Cortex_GetSdkVersion(SDK_Version);
    std::cout << "Using SDK Version: "
              << SDK_Version[1] << "."
              << SDK_Version[2] << "."
              << SDK_Version[3] << ".\n";

    Cortex_SetErrorMsgHandlerFunc();
    Cortex_SetDataHandlerFunc();

    if (argc == 1)
    {
        retval = Cortex_Initialize((char*)"", (char*)NULL);
    }
    else
    if (argc == 2)
    {
        retval = Cortex_Initialize(argv[1], (char*)NULL);
    }
    else
    if (argc == 3)
    {
        retval = Cortex_Initialize(argv[1], argv[2]);
    }

    if (retval != RC_Okay)
    {
        std::cerr << "Error: Unable to initialize ethernet communication\n";
        return -1;
    }

    retval = Cortex_GetHostInfo(&Cortex_HostInfo);

	if (retval != RC_Okay
     || !Cortex_HostInfo.bFoundHost)
    {
        std::cerr << "Cortex not found.\n";
        return -1;
    }
    else
    {
        std::cout << "Found " << Cortex_HostInfo.szHostProgramName
                  << " Version " << Cortex_HostInfo.HostProgramVersion[1]
                  << "." << Cortex_HostInfo.HostProgramVersion[2]
                  << "." << Cortex_HostInfo.HostProgramVersion[3]
                  << " at " << Cortex_HostInfo.HostMachineAddress[0]
                  << "." << Cortex_HostInfo.HostMachineAddress[1]
                  << "." << Cortex_HostInfo.HostMachineAddress[2]
                  << "." << Cortex_HostInfo.HostMachineAddress[3]
                  << " (" << Cortex_HostInfo.szHostMachineName << ")\n";
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

    retval = Cortex_Exit();
    std::cout << "Press any key to continue...";
    key = getchar();

    return 0;
}
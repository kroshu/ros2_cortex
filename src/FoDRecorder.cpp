#include <windows.h> // for the Sleep function
#include <stdio.h>   // for the printf function
#include <conio.h>
#include <cstdio>

#include "Cortex.h"
#include "rapidjson/document.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/writer.h"

rapidjson::Document json_doc; //TODO not global...

void MyErrorMsgHandler(int iLevel, char *szMsg)
{
    const char *szLevel;

    if (iLevel == VL_Debug)
    {
        szLevel = "Debug";
    }
    else
    if (iLevel == VL_Info)
    {
        szLevel = "Info";
    }
    else
    if (iLevel == VL_Warning)
    {
        szLevel = "Warning";
    }
    else
    if (iLevel == VL_Error)
    {
        szLevel = "Error";
    }

    printf("    %s: %s\n", szLevel, szMsg);
}

void PrintMarkerData(tMarkerData* markerData, int nMarkers, rapidjson::Value& markers, rapidjson::Document::AllocatorType& allocator){
    for (int i=0 ; i<nMarkers ; i++)
    {
        rapidjson::Value marker(rapidjson::kObjectType);
        marker.AddMember("x", markerData[i][0], allocator);
        marker.AddMember("y", markerData[i][1], allocator);
        marker.AddMember("z", markerData[i][2], allocator);
        markers.PushBack(marker, allocator);
    }
}

void PrintSegmentData(tSegmentData* segmentData, int nSegments, rapidjson::Value& segments, rapidjson::Document::AllocatorType& allocator){
    for (int i=0 ; i<nSegments ; i++)
    {
        rapidjson::Value segment(rapidjson::kObjectType);
        segment.AddMember("x", segmentData[i][0], allocator);
        segment.AddMember("y", segmentData[i][1], allocator);
        segment.AddMember("z", segmentData[i][2], allocator);
        segment.AddMember("aX", segmentData[i][3], allocator);
        segment.AddMember("aY", segmentData[i][4], allocator);
        segment.AddMember("aZ", segmentData[i][5], allocator);
        segment.AddMember("length", segmentData[i][6], allocator);
        segments.PushBack(segment, allocator);
    }
}

void PrintForceData(tForceData* forceData, int nForceSamples, int nForcePlates, rapidjson::Value& forces, rapidjson::Document::AllocatorType& allocator){
    for (int iSample=0; iSample<nForceSamples; iSample++)
    {
        for (int iPlate=0; iPlate<nForcePlates; iPlate++)
        {
            rapidjson::Value force(rapidjson::kObjectType);
            force.AddMember("forcePlate", iPlate, allocator);
            force.AddMember("x", *forceData[0], allocator);
            force.AddMember("y", *forceData[1], allocator);
            force.AddMember("z", *forceData[2], allocator);
            force.AddMember("fX", *forceData[3], allocator);
            force.AddMember("fY", *forceData[4], allocator);
            force.AddMember("fZ", *forceData[5], allocator);
            force.AddMember("mZ", *forceData[6], allocator);
            forces.PushBack(force, allocator);
            forceData++;
        }
    }
}

void PrintBodyDatas(sBodyData* bodyData, int nBodies, rapidjson::Value& bodies, rapidjson::Document::AllocatorType& allocator){
    int i=0;
    for (int iBody=0; iBody < nBodies; iBody++)
    {
        sBodyData *Body = &bodyData[iBody];
        rapidjson::Value body(rapidjson::kObjectType);
        // TODO check whether this is safe
        body.AddMember("name", rapidjson::StringRef(Body->szName), allocator);

        body.AddMember("nMarkers", Body->nMarkers, allocator);
        rapidjson::Value markers(rapidjson::kArrayType);
        PrintMarkerData(Body->Markers, Body->nMarkers, markers, allocator);
        body.AddMember("markers", markers, allocator);
        body.AddMember("fAvgMarkerResidual", Body->fAvgMarkerResidual, allocator);

        body.AddMember("nSegments", Body->nSegments, allocator);
        rapidjson::Value segments(rapidjson::kArrayType);
        PrintSegmentData(Body->Segments, Body->nSegments, segments, allocator);
        body.AddMember("segments", segments, allocator);

        body.AddMember("nDofs", Body->nDofs, allocator);
        rapidjson::Value dofs(rapidjson::kArrayType);
        for (i=0 ; i<Body->nDofs ; i++)
        {
            dofs.PushBack(Body->Dofs[i], allocator);
        }
        body.AddMember("dofs", dofs, allocator);
        body.AddMember("fAvgDofResidual", Body->fAvgDofResidual, allocator);
        body.AddMember("nIterations", Body->nIterations, allocator);

        body.AddMember("encoderZoom", Body->ZoomEncoderValue, allocator);
        body.AddMember("encoderFocus", Body->FocusEncoderValue, allocator);
        body.AddMember("encoderIris", Body->IrisEncoderValue, allocator);
        rapidjson::Value camTrackParams(rapidjson::kObjectType);
        camTrackParams.AddMember("offsetX", Body->CamTrackParams[0], allocator);
        camTrackParams.AddMember("offsetY", Body->CamTrackParams[1], allocator);
        camTrackParams.AddMember("offsetZ", Body->CamTrackParams[2], allocator);
        camTrackParams.AddMember("offsetAngleX", Body->CamTrackParams[3], allocator);
        camTrackParams.AddMember("offsetAngleY", Body->CamTrackParams[4], allocator);
        camTrackParams.AddMember("offsetAngleZ", Body->CamTrackParams[5], allocator);
        camTrackParams.AddMember("videoWidth", Body->CamTrackParams[6], allocator);
        camTrackParams.AddMember("videoHeight", Body->CamTrackParams[7], allocator);
        camTrackParams.AddMember("opticalCenterX", Body->CamTrackParams[8], allocator);
        camTrackParams.AddMember("opticalCenterY", Body->CamTrackParams[9], allocator);
        camTrackParams.AddMember("fovX", Body->CamTrackParams[10], allocator);
        camTrackParams.AddMember("fovY", Body->CamTrackParams[11], allocator);
        camTrackParams.AddMember("pixelAspect", Body->CamTrackParams[12], allocator);
        camTrackParams.AddMember("firstCoefficient", Body->CamTrackParams[13], allocator);
        body.AddMember("camTrackParams", camTrackParams, allocator);

        body.AddMember("nEvents", Body->nEvents, allocator);
        rapidjson::Value events(rapidjson::kArrayType);
        for (i=0 ; i<Body->nEvents ; i++)
        {
            events.PushBack(rapidjson::StringRef(Body->Events[i]), allocator);
        }
        body.AddMember("events", events, allocator);

        bodies.PushBack(body, allocator);
    }
    
}

void PrintAnalogData(sAnalogData& analogData, rapidjson::Value& adValue, rapidjson::Document::AllocatorType& allocator){
    adValue.AddMember("nAnalogChannels", analogData.nAnalogChannels, allocator);
    adValue.AddMember("nAnalogSamples", analogData.nAnalogSamples, allocator);

    int nSamples = analogData.nAnalogSamples;
    int nChannels = analogData.nAnalogChannels;
    short *pSample = analogData.AnalogSamples;
    rapidjson::Value analogSamples(rapidjson::kArrayType);
    for (int iSample=0 ; iSample<nSamples ; iSample++)
    {
        for (int iChannel=0 ; iChannel<nChannels ; iChannel++)
        {
            rapidjson::Value sample(rapidjson::kObjectType);
            sample.AddMember("channel", iChannel, allocator);
            sample.AddMember("value", *pSample, allocator);
            analogSamples.PushBack(sample, allocator);
            pSample++;
        }
    }
    adValue.AddMember("analogSamples", analogSamples, allocator);

    adValue.AddMember("nForcePlates", analogData.nForcePlates, allocator);
    adValue.AddMember("nForceSamples", analogData.nForceSamples, allocator);
    rapidjson::Value forces(rapidjson::kArrayType);
    PrintForceData(analogData.Forces, analogData.nForceSamples, analogData.nForcePlates, forces, allocator);
    adValue.AddMember("forces", forces, allocator);

    adValue.AddMember("nAngleEncoders", analogData.nAngleEncoders, allocator);
    adValue.AddMember("nAngleEncoderSamples", analogData.nAngleEncoderSamples, allocator);

    int nAngleEncoders = analogData.nAngleEncoders;
	int nAngleEncoderSamples = analogData.nAngleEncoderSamples;
    double* AngleEncoderSamples = analogData.AngleEncoderSamples;
    double *ptr = AngleEncoderSamples;
    rapidjson::Value angleEncoderSamples(rapidjson::kArrayType);
    for (int iSample=0 ; iSample<nAngleEncoderSamples ; iSample++)
	{
		for (int i=0 ; i<nAngleEncoders ; i++)
	    {
		    rapidjson::Value sample(rapidjson::kObjectType);
            sample.AddMember("encoder", i, allocator);
            sample.AddMember("value", *ptr, allocator);
            angleEncoderSamples.PushBack(sample, allocator);
			ptr++;
		}
	}
    adValue.AddMember("angleEncoderSamples", angleEncoderSamples, allocator);
}

void PrintFrameOfData(sFrameOfData *FrameOfData)
{
    rapidjson::Document::AllocatorType& allocator = json_doc.GetAllocator(); // TODO maybe instead get as param??
    rapidjson::Value frame(rapidjson::kObjectType);
    frame.AddMember("frame", FrameOfData->iFrame, allocator);
    frame.AddMember("frameDelay", FrameOfData->fDelay, allocator);
    frame.AddMember("nBodies", FrameOfData->nBodies, allocator);

    rapidjson::Value bodies(rapidjson::kArrayType);
    PrintBodyDatas(FrameOfData->BodyData, FrameOfData->nBodies, bodies, allocator);
    frame.AddMember("bodies", bodies, allocator);

    frame.AddMember("nUnidentifiedMarkers", FrameOfData->nUnidentifiedMarkers, allocator);
    rapidjson::Value uiMarkers(rapidjson::kArrayType);
    PrintMarkerData(FrameOfData->UnidentifiedMarkers, FrameOfData->nUnidentifiedMarkers, uiMarkers, allocator);
    frame.AddMember("unidentifiedMarkers", uiMarkers, allocator);

    rapidjson::Value analogData(rapidjson::kObjectType);
    PrintAnalogData(FrameOfData->AnalogData, analogData, allocator);
    frame.AddMember("analogData", analogData, allocator);

    sRecordingStatus *RC = &FrameOfData->RecordingStatus;
    rapidjson::Value rcstatus_value(rapidjson::kObjectType);
    rcstatus_value.AddMember("recording", RC->bRecording, allocator);
    rcstatus_value.AddMember("firstFrame", RC->iFirstFrame, allocator);
    rcstatus_value.AddMember("lastFrame", RC->iLastFrame, allocator);
    rcstatus_value.AddMember("captureFileName", rapidjson::StringRef(RC->szFilename), allocator);
    frame.AddMember("recordingStatus", rcstatus_value, allocator);

    rapidjson::Value timecode_value(rapidjson::kObjectType);
    sTimeCode* TC = &FrameOfData->TimeCode;
    timecode_value.AddMember("hours", TC->iHours, allocator);
    timecode_value.AddMember("minutes", TC->iMinutes, allocator);
    timecode_value.AddMember("seconds", TC->iSeconds, allocator);
    timecode_value.AddMember("frames", TC->iFrames, allocator);
    timecode_value.AddMember("standard", TC->iStandard, allocator);
    frame.AddMember("timeCode", timecode_value, allocator);
    
    json_doc["framesArray"].PushBack(frame, allocator);
}

void MyDataHandler(sFrameOfData* FrameOfData)
{
    static int Count=0;
    if (Count >= 7231) return;
    if (Count >= 7230)
    {
        FILE* fp = fopen("CaptureWithPlots1.json", "wb");

        char writeBuffer[65536];
        rapidjson::FileWriteStream os(fp, writeBuffer, sizeof(writeBuffer));

        rapidjson::Writer<rapidjson::FileWriteStream> writer(os);
        json_doc.Accept(writer);

        fclose(fp);
        Count++;
        return;
    }

    PrintFrameOfData(FrameOfData);
    Count++;
}

int main(int argc, char* argv[])
{
    sHostInfo Cortex_HostInfo;
    int retval;
    unsigned char SDK_Version[4];
    char key;
    int i;
    sBodyDefs*    pBodyDefs=NULL;
    sFrameOfData* pFrameOfData=NULL;
    sFrameOfData  MyCopyOfFrame;
    
    json_doc.SetObject();
    rapidjson::Document::AllocatorType& allocator = json_doc.GetAllocator();
    rapidjson::Value framesArray(rapidjson::kArrayType);
    json_doc.AddMember("framesArray", framesArray, allocator);
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);

    memset(&MyCopyOfFrame, 0, sizeof(sFrameOfData));

    printf("Usage: ClientTest <Me> <Cortex>\n");
    printf("       Me = My machine name or its IP address\n");
    printf("       Cortex = Cortex's machine name or its IP Address\n");

    for (i=0 ; i<argc ; i++)
        printf(" %s", argv[i]);
    printf("\n");

    printf("----------\n");

    Cortex_SetVerbosityLevel(VL_Info); //(VL_Debug);

    Cortex_GetSdkVersion(SDK_Version);
    printf("Using SDK Version %d.%d.%d\n",
        SDK_Version[1],
        SDK_Version[2],
        SDK_Version[3]);


    Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
    Cortex_SetDataHandlerFunc(MyDataHandler);

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
        printf("Error: Unable to initialize ethernet communication\n");
        goto DONE; //goto :||||
    }

    retval = Cortex_GetHostInfo(&Cortex_HostInfo);

	if (retval != RC_Okay
     || !Cortex_HostInfo.bFoundHost)
    {
        printf("\n");
        printf("Cortex not found.\n");
    }
    else
    {
        printf("\n");
        printf("Found %s Version %d.%d.%d at %d.%d.%d.%d (%s)\n",
            Cortex_HostInfo.szHostProgramName,
            Cortex_HostInfo.HostProgramVersion[1],
            Cortex_HostInfo.HostProgramVersion[2],
            Cortex_HostInfo.HostProgramVersion[3],

            Cortex_HostInfo.HostMachineAddress[0],
            Cortex_HostInfo.HostMachineAddress[1],
            Cortex_HostInfo.HostMachineAddress[2],
            Cortex_HostInfo.HostMachineAddress[3],
            Cortex_HostInfo.szHostMachineName);
    }

    while (1)
    {
        key = getch();
        if (key == 'Q'
         || key == 'q')
        {
            break;
        }
    }

DONE:

    retval = Cortex_Exit();
    printf("Press any key to continue...");
    key = getch();

    return 0;
}

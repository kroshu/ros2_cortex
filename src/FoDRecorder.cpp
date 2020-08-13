#include <windows.h> // for the Sleep function
#include <stdio.h>   // for the printf function
#include <conio.h>
#include <cstdio>

#include "Cortex.h"
#include "rapidjson/document.h"
#include "rapidjson/filewritestream.h"
#include "rapidjson/writer.h"

rapidjson::Document json_doc;

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

void PrintFrameOfData(sFrameOfData *FrameOfData)
{
    int iBody;
    int i;

    rapidjson::Document::AllocatorType& allocator = json_doc.GetAllocator(); //TODO maybe instead get as param??
    rapidjson::Value frame(rapidjson::kObjectType);
    frame.AddMember("frame", FrameOfData->iFrame, allocator);
    frame.AddMember("frameDelay", FrameOfData->fDelay, allocator);
    frame.AddMember("nBodies", FrameOfData->nBodies, allocator);
    //TODO bodies

    frame.AddMember("nUnidentifiedMarkers", FrameOfData->nUnidentifiedMarkers, allocator);
    //TODO unid markers

    //TODO analog data

    rapidjson::Value timecode_value(rapidjson::kObjectType);
    sTimeCode* TC = &FrameOfData->TimeCode;
    timecode_value.AddMember("hours", TC->iHours, allocator);
    timecode_value.AddMember("minutes", TC->iMinutes, allocator);
    timecode_value.AddMember("seconds", TC->iSeconds, allocator);
    timecode_value.AddMember("frames", TC->iFrames, allocator);
    switch (TC->iStandard)
    {
        case 1: timecode_value.AddMember("standard", "SMPTE", allocator); break;
        case 2: timecode_value.AddMember("standard", "FILM", allocator); break;
        case 3: timecode_value.AddMember("standard", "EBU", allocator); break;
        case 4: timecode_value.AddMember("standard", "SYSTEMCLOCK", allocator); break;
    }
    frame.AddMember("timeCode", timecode_value, allocator);

    sRecordingStatus *RC = &FrameOfData->RecordingStatus;
    rapidjson::Value rcstatus_value(rapidjson::kObjectType);
    rcstatus_value.AddMember("recording", RC->bRecording, allocator);
    rcstatus_value.AddMember("recordFirstFrame", RC->iFirstFrame, allocator);
    rcstatus_value.AddMember("recordLastFrame", RC->iLastFrame, allocator);
    //TODO RC->szFileName

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
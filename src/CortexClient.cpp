#include <iostream>

#include "CortexClient.hpp"

CortexClient::CortexClient(const std::string& capture_file_name){
	cortex_mock_ = CortexMock(capture_file_name);
}

CortexClient::~CortexClient(){
    freeFrameOfData(current_fod_);
}

void CortexClient::freeFrameOfData(sFrameOfData& fod){
	int n_bodies = fod.nBodies;
	if(n_bodies > 0){
		for (int i_body = 0; i_body < n_bodies; ++i_body) {
			sBodyData& i_body_data = fod.BodyData[i_body];
			if(i_body_data.nMarkers > 0) delete [] i_body_data.Markers;
			if(i_body_data.nSegments > 0) delete [] i_body_data.Segments;
			if(i_body_data.nDofs > 0) delete [] i_body_data.Dofs;
			int n_events = i_body_data.nEvents;
			for (int i_event = 0; i_event < n_events; ++i_event) {
				if(i_body_data.Events[i_event] != nullptr) delete [] i_body_data.Events[i_event];
			}
			if(i_body_data.nEvents > 0) delete [] i_body_data.Events;
		}
	}

	if(fod.nUnidentifiedMarkers > 0) delete [] fod.UnidentifiedMarkers;
	if(fod.AnalogData.nAnalogSamples > 0 && fod.AnalogData.nAnalogChannels > 0) delete [] fod.AnalogData.AnalogSamples;
	if(fod.AnalogData.nForceSamples > 0 && fod.AnalogData.nForcePlates > 0) delete [] fod.AnalogData.Forces;
	if(fod.AnalogData.nAngleEncoderSamples > 0 && fod.AnalogData.nAngleEncoders > 0) delete [] fod.AnalogData.AngleEncoderSamples;
}

void CortexClient::run(){
	CortexMock cortex_mock("CaptureWithPlots1.json");
	cortex_mock.initialize("127.0.0.1", "127.0.0.1");
}
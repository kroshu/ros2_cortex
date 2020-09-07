#include <iostream>
#include <vector>

#include "CortexClient.hpp"

class SimpleFodPrinter: public CortexClient{

public:
    SimpleFodPrinter(const std::string& file_name):CortexClient{file_name}{}
    void dataHandlerFunc_(sFrameOfData* fod){
        cortex_mock_.copyFrame(fod, &current_fod_);
        std::cout << "Frame " << current_fod_.iFrame << std::endl;
        std::cout << "Number of unidentified markers " << current_fod_.nUnidentifiedMarkers << std::endl;
    }

    const std::vector<std::string> verb_levels = {"None", "Error", "Warning", "Info", "Debug"};

    void errorMsgHandlerFunc_(int i_level, char* error_msg){
        std::cerr << verb_levels[i_level] << ": " << error_msg << std::endl;
    }
};

const static std::string file_name("CaptureWithPlots1.json");
static SimpleFodPrinter printer(file_name);

void dataHandlerFunc(sFrameOfData* fod){
    printer.dataHandlerFunc_(fod);
}

void errorMsgHandlerFunc(int i_level, char* error_msg){
    printer.errorMsgHandlerFunc_(i_level, error_msg);
}

int main(int argc, char const *argv[])
{
    printer.setDataHandlerFunc(dataHandlerFunc);
    printer.setErrorMsgHandlerFunc(errorMsgHandlerFunc);
    printer.run();
    
    return 0;
}

#include <iostream>

#include "CortexClient.hpp"


class SimpleFodPrinter: public CortexClient{

public: SimpleFodPrinter(const std::string& file_name):CortexClient{file_name}{}
    void dataHandlerFunc_(sFrameOfData* fod){
        sFrameOfData copy_fod;
        cortex_mock_.copyFrame(fod, &copy_fod);
        std::cout << "Frame " << fod->iFrame << std::endl;
        std::cout << "Number of unidentified markers " << fod->nUnidentifiedMarkers << std::endl;
    }

};

const static std::string file_name("CaptureWithPlots1.json");
static SimpleFodPrinter printer(file_name);

void dataHandlerFunc(sFrameOfData* fod){
    printer.dataHandlerFunc_(fod);
}

int main(int argc, char const *argv[])
{
    printer.setdataHandlerFunc(dataHandlerFunc);
    printer.run();
    
    return 0;
}

#include <iostream>

#include "CortexClient.hpp"

class SimpleFodPrinter: public CortexClient{
    void dataHandlerFunc(sFrameOfData& fod){
        std::cout << "Frame " << fod.iFrame << std::endl;
        std::cout << "Number of unidentified markers " << fod.nUnidentifiedMarkers << std::endl;
    }
};

int main(int argc, char const *argv[])
{
    // std::string addr("127.0.0.1");
    // const int port = 30001;
    SimpleFodPrinter printer;
    printer.run();
    
    return 0;
}

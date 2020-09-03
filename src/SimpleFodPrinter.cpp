#include <iostream>

#include "CortexClient.hpp"

class SimpleFodPrinter: public CortexClient{
    void dataHandlerFunc_(sFrameOfData* fod){
        std::cout << "Frame " << fod->iFrame << std::endl;
        std::cout << "Number of unidentified markers " << fod->nUnidentifiedMarkers << std::endl;
    }
};

int main(int argc, char const *argv[])
{
    SimpleFodPrinter printer;
    printer.run();
    
    return 0;
}

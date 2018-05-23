#ifndef MANIPULATE_DATA_H
#define MANIPULATE_DATA_H

#include <string>
#include <ctype.h>
#include <string.h>

#include "compass_old/compass_data.h"

//Class to hold methods that will do work on the imported data
class ManipulateData {
public:
    Compass_Data ParseData(std::string);

    std::string LoadData();

    double PullData(std::string, int);

    void SendData(Compass_Data);
    
};
//end ManipulateData class

#endif

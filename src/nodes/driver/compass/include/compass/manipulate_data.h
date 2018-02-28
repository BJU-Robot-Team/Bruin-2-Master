#include <string>
#include <ctype.h>
#include <string.h>

#include "compass/compass_data.h"

//Class to hold methods that will do work on the imported data
class ManipulateData {
      public:
        Compass_Data ParseData(string); 

        string LoadData();

        double PullData(string, int);

        void SendData(Compass_Data);

};//end ManipulateData class

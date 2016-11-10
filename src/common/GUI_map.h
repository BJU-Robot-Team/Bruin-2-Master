#IFNDEF MAP_H 
#define MAP_H

#include <string> 
#include <cmath> 
#include <algorithm> 

//Stations are locations that we can travel from and to. Station struct defines
//  where on the image a station is and what it is called.
struct Station {
    string name;
    int x,y; //location on the image from top left corner
}

//The map object contains a list of stations, their location on an image, and 
//  waypoints needed  
struct Map {
    string image_path;
    vector<station> stations;
}



#endif 

#ifndef WAYPOINT_MAP_H
#define WAYPOINT_MAP_H

//this file contains definitions for a waypoint map. a set of stations with one
//  way paths between each one.

#include <string>
#include <vector>

//waypoints are GPS locations that describe a section of the path the vehicle
//  will travel
struct Waypoint {
    std::string name;
    float latitude, longitude;
    float curvature; //how much to curve the path while aproching this waypoint
    float precision; //minimum distance (m) from the waypoint considered an arival
    bool stop_sign; //weather this waypoint is at a stop sign
    
    Waypoint() {
        name = "";
        stop_sign = false;
    }
    
    Waypoint(std::string a_name, float a_latitude, float a_longitude,
            float a_curvature, float a_precision, float a_stop_sign) :
            name(a_name), latitude(a_latitude), longitude(a_longitude), curvature(
                    a_curvature), precision(a_precision), stop_sign(a_stop_sign) {
    }
};

//WaypointPaths are one way collections of waypoints that describe how to get
//  from a specific start station to a specific end station
struct WaypointPath {
    std::string start_station, end_station;
    std::vector<Waypoint> waypoints;
};

//the waypoint map holds all paths to all stations
class WaypointMap {
private:
    //list of stations and their index in the station matrix
    std::vector<std::string> station_index;

    //station matrix holding all possible paths. first index is the start
    //  station. second index is the end station
    std::vector<std::vector<WaypointPath> > station_matrix;

    //open and parse the data in one CSV file
    bool parseCSVData(std::string file_path, bool file_type_waypoints);

    //get the index for a given station pair
    void findStationIndexes(std::string start_station, std::string end_station,
            int& start_station_index, int& end_station_index);

public:
    
    WaypointMap() {
    }
    ;
    WaypointMap(std::string map_folder_path);

    ~WaypointMap() {
    }
    ;

    //loads all relevant map files from a map's folder
    bool loadCSVMapFormat(std::string map_folder_path);

    //returns a path given two stations
    WaypointPath *getWaypointList(std::string start_station,
            std::string end_station);
};

#endif

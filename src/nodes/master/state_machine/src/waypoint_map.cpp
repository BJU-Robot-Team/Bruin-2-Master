//Contains the code for the waypoint map.

#include <string>
#include <vector>

#include <iostream>     // std::ios, std::istream, std::cout
#include <fstream>      // std::filebuf
#include <dirent.h>     // DIR type, opendir, readdir

#include "state_machine/waypoint_map.h"
#include "csv.h"


WaypointMap::WaypointMap(std::string map_folder_path) {
    bool loaded = loadCSVMapFormat(map_folder_path);

    //if something went wrong, print the error to cout
    if (!loaded) {
      std::cout << "The desired CSV file path and files were not loaded" << std::endl;
    }
}


//goes through all files in a map format folder and loads their data.
//  takes a folder path string
//  returns bool. true means success
bool WaypointMap::loadCSVMapFormat(std::string map_folder_path) {
    bool parse_success;

    //parse the stations file first
    parse_success = parseCSVData(map_folder_path+"/stations.csv", false);

    //if we cannot parse the stations file we cannot go on.
    if (parse_success == false) {
        std::cout << "not loaded" << std::endl;
        return parse_success; //TODO handle error.
    }

    //list files in the map directory and load them

    //directory list code aquired from http://stackoverflow.com/questions/306533
    //  /how-do-i-get-a-list-of-files-in-a-directory-in-c from awnser Kloberdanz

    //create a pointer that wil point to the directory
    DIR *dpdf;

    struct dirent *epdf;

    //use the opendir command to get a pointer to the *map_folder_path*
    //seems to work similar to fopen, except it's for directories to iterate over
    //files
    dpdf = opendir(map_folder_path.c_str());

    //if the directory exists after calling opendir, parse the information inside
    if (dpdf != NULL){
        //set the dirent struct pointer equal to the open directory
        epdf = readdir(dpdf);
        //while there is another file in the directory, continue looping
        while (epdf){
            std::string filename = epdf->d_name;

            //we already parsed the stations.csv file so we skip it
            if (filename == "stations.csv") {
                continue;

            //parse every file that ends in .csv
            //  (minus the stations.csv we skiped above)
            } else if (filename.substr(filename.length()-4) == ".csv") {
                parseCSVData(map_folder_path+"/"+filename, true);
            } else { //skip all other files
                continue;
            }
            epdf = readdir(dpdf);
        }
    } else {
      //TODO: this should probably be changed to the ROS_DEBUG_STREAM
      std::cout << "The directory name provided to the CSV parser method was invalid" << std::endl;
    }
    //close the open directory just as you would do with a normal file
    closedir(dpdf);

    return parse_success;
}


//parses a given CSV file's contents.
//  takes a filepath string and a bool indicating which file type this is
//  returns a bool. true indicates success
//  TODO: we need to do a lot of validation
//     * validate that both start and end stations are not the same
//     * validate that we get the bare essential data for a waypoint
//     * maybe validate that a station isn't repeated in the station.csv file
//     * maybe make sure path is a valid file path
//  TODO: we also need debug messages when something fails like file opening.
bool WaypointMap::parseCSVData(std::string file_path, bool file_type_waypoints) {

    std::filebuf file;

    //open file and add it to a stream object
    if ( file.open(file_path, std::ios::in) ) {
        std::istream filestream(&file);

        //handle stations.csv file format
        if (file_type_waypoints == false) {
            //setup csv parser reads 1 column
            io::CSVReader<1> csv_reader(file_path, filestream);

            //staion name
            std::string station;

            //read each line getting the station name
            while(csv_reader.read_row(station)) {
                station_index.push_back(station);
            }

        //normal waypoints path file format
        } else {
            //setup csv parser
            //first line has only 2 columns
            io::CSVReader<2> station_reader(file_path, filestream);
            //rest of the file has 6 columns
            io::CSVReader<6> waypoint_reader(file_path, filestream);
            waypoint_reader.set_file_line(2);

            //read first line which describes source and destination stations
            std::string start_station, end_station;
            station_reader.read_row(start_station, end_station);

            //read the rest of the lines which are each waypoints
            std::string comment;
            double latitude, longitude, curvature, precision;
            int stop_sign_value=0; //value is optional so we pre initialize it

            std::vector<Waypoint> waypoints;

            //read each line and get the first column's info then add it to the
            //  station index
            while(waypoint_reader.read_row(comment, latitude, longitude, curvature, precision, stop_sign_value)) {

                bool stop_sign;
                if (stop_sign_value >= 1) {
                    stop_sign = true;
                } else {
                    stop_sign = false;
                }

                //make a waypoint object from the read data
                Waypoint point(comment, latitude, longitude, curvature,
                              precision, stop_sign);

                //add the waypoints to the end of our list
                waypoints.push_back(point);
            }

            //create a path object from the data read in.
            WaypointPath wp_path;
            wp_path.start_station = start_station;
            wp_path.end_station = end_station;
            wp_path.waypoints = waypoints;

            //add the path to the station matrix

            //starting and ending station's indexes
            int start_station_index = -1;
            int end_station_index = -1;

            findStationIndexes(start_station, end_station,
                    start_station_index, end_station_index);

            //check if we found both stations
            if (start_station_index == -1 || end_station_index == -1) {
                //one of the stations could not be found so we end without saving
                file.close();
                return false;
            } else {
                //we found the index's so we can now add the path to the matrix
                station_matrix[start_station_index][end_station_index] = wp_path;
            }
        }

        //close the file
        file.close();
        return true;
    } else {
        throw std::runtime_error("Waypoint map loading is missing a file. path: "+file_path);
    }
    return false;
}


//returns a waypoint path given two stations
WaypointPath *WaypointMap::getWaypointList(std::string start_station, std::string end_station) {
    //starting and ending station's indexes
    int start_station_index = -1;
    int end_station_index = -1;

    findStationIndexes(start_station, end_station,
                 start_station_index, end_station_index);

    //check if we found both stations
    if (start_station_index == -1 || end_station_index == -1) {
         //one of the stations could not be found so we cannot return a path for it
         return nullptr;
     } else {
         //we found the index's so we can now add the path to the matrix
         return nullptr;//station_matrix[start_station_index][end_station_index];
     }
}


void WaypointMap::findStationIndexes(std::string start_station, std::string end_station,
                            int& start_station_index, int& end_station_index){
    //go through the station index list and find what index the start
    //  and end stations have
    int i = 0;
    for (auto it=station_index.begin(); it != station_index.end(); it++,i++) {

        //set station index if we found it.
        if (*it == start_station) {
            start_station_index = i;
        } else if (*it == end_station) {
            end_station_index = i;
        }

        //end the loop early if we found both
        if (start_station_index != -1 && end_station_index != -1) {
            break;
        }
    } //end for loop

}

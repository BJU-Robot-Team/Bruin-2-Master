#include <iostream>
#include <fstream>
 
#include <stdio.h>
#include <assert.h>
#include <vector>
#include <string>

//evil way to gain access to private members but useful for testing purpose 
//#define private public
#include "state_machine/waypoint_map.h"

//create test csv files
void creatCSVFile(std::string name, std::vector<std::string> lines) {
    std::ofstream file (name.c_str());

    std::vector<std::string>::iterator it;
    for (it=lines.begin(); it != lines.end(); it++) { 
        file << *it << std::endl;
    }

    file.close();
}

int main(void) {
    //run tests against the functionality of the waypoint map code

    //test Waypoint Struct
    Waypoint test1("test1", 86.2234, 32.2233, 1, 1.5, false);

    assert(test1.name == "test1");
    assert(test1.latitude == 86.2234);
    assert(test1.longitude == 32.2233);
    assert(test1.curvature == 1);
    assert(test1.precision == 1.5);
    assert(test1.stop_sign == false);

    //test WaypointPath struct
    Waypoint test2("test2", 86.2434, 32.2233, 1, 1.5, false);
    Waypoint test3("test3", 86.2434, 32.2333, 1, 1.5, false);

    std::vector<Waypoint> wps;
    wps.push_back(test1);
    wps.push_back(test2);
    wps.push_back(test3);

    WaypointPath test_path;
    test_path.waypoints = wps;
    test_path.start_station = "Test station 1";
    test_path.end_station = "Test station 2";

    //test WaypointMap struct functionality
    //-create csv files in the current directory for testing the parser
    std::vector<std::string> lines;
    lines.push_back("\"Test station 1\",34,34");
    lines.push_back("\"Test station 2\",60,34");
    
    //creatCSVFile("stations.csv", lines);
    
    lines.clear();

    lines.push_back("\"Test station 1\",\"Test station 2\"");
    lines.push_back("start,86.2234, 32.2233, 1, 1.5");
    lines.push_back("end, 86.2434, 32.2233, 1, 1.5, false");
    
    //creatCSVFile("test1 to test2.csv", lines);
    
    lines.clear();

    lines.push_back("\"Test station 1\",\"Test station 2\"");
    lines.push_back("start, 86.2434, 32.2233, 1, 1.5, false");
    lines.push_back("end,86.2234, 32.2233, 1, 1.5, true");
    
    //creatCSVFile("test2 to test1.csv", lines);

    //-test parsing the csv files 'mannually'
    //  (remember these are private members but we have a hack at the 
    //  top of this file)
    WaypointMap test_map1;

    //  allways have to load the stations file first
    test_map1.parseCSVData("stations.csv", false);

    //  test waypoint files
    test_map1.parseCSVData("test1 to test2.csv", true);
    test_map1.parseCSVData("test2 to test1.csv", true);

    //-test parsing 'automatically'
    WaypointMap test_map2;
    
    test_map2.loadCSVMapFormat(""); //path to current working directory

    //-clean up test csv files from CWD.
    //remove("stations.csv");
    //remove("test1 to test2.csv");
    //remove("test2 to test1.csv");

}

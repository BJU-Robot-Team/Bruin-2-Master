
#include <iostream>
#include <fstream>
#include <ctime>
#include <string>

#include "topic_logger/ros_interface.h"

//globals (yuck) for the various file objects
//TODO study the callback functions and figure out how to pass the data without globals
std::ofstream relay_file;
std::ofstream compass_file;
std::ofstream gps_file;
std::ofstream camera_file;



//Write to log file
void writeData(std::ofstream& file, std::string data) {
    //get current time
    std::time_t t = time(0);
    struct tm * now = localtime( & t );

    //concatinate a nice date string
    std::string date_str = "["
                         //TODO add time here
                         + std::to_string(now->tm_year + 1900) + '-'
                         + std::to_string(now->tm_mon + 1) + '-'
                         + std::to_string(now->tm_mday)
                         + "]";

    file << date_str << " " << data << std::endl;
}



//###Define ROS message callbacks so we can log messages as they come in.

void relay_callback(const relay_board::RelayDataMsg& relayStatusMessage) {
    std::string data = "";

    writeData(relay_file, data);
}


void compass_callback(const compass::CompassDataMsg& compassMessage) {
    std::string data = "Compass heading: " + std::to_string(compassMessage.heading);

    writeData(compass_file, data);
}

void gps_callback(const sensor_msgs::NavSatFix& gpsMessage) {
    std::string data = "Compass latitude: " + std::to_string(gpsMessage.latitude) + '\n'
                     + "	longitude: " + std::to_string(gpsMessage.longitude) +'\n'
                     + "	altitude: " + std::to_string(gpsMessage.altitude);// + '\n'
//                     + "	status: " + std::to_string(gpsMessage.status);

    writeData(gps_file, data);
}

void camera_callback(const camera_node::CameraDataMsg& cameraMessage) {
    std::string data = "Camera direction: " + std::to_string(cameraMessage.direction) + '\n'
                     + "	distance: " + std::to_string(cameraMessage.distance) + '\n'
                     + "	valid: " + std::to_string(cameraMessage.tracking);
    
}




int main(int argc, char **argv) {

    //function defined in ros_interface header
    startROS(argc, argv);

    //setup ros interface, state machine, and vehicle data  objects
    ROSInterface ros_interface( &relay_callback, &compass_callback, &gps_callback, &camera_callback);

    //open log files
    
    //cwd is /home/<user>/.ros
    //base path goes from there to the log folder
    //TODO maybe put it one more directory down like "sensor_data"
    std::string base_path = "log/latest/";

    relay_file = std::ofstream(base_path+"relay_board_msgs.log", std::ios_base::app);
    compass_file = std::ofstream(base_path+"compass_msgs.log", std::ios_base::app);
    gps_file = std::ofstream(base_path+"gps_msgs.log", std::ios_base::app);
    camera_file = std::ofstream(base_path+"camera_msgs.log", std::ios_base::app);


    //start main loop (just to keep the program running while ROS is running)
    while (ros_interface.isNodeRunning()) {
        ros::spinOnce();  // Need to spin if we use callbacks, and to let ROS know we are alive
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    //close the opened files
    relay_file.close();
    compass_file.close();
    camera_file.close();
    
}

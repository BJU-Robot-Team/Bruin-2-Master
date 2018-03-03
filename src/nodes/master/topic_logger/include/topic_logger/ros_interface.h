// State machine ROS Details
#ifndef ROS_INTERFACE_H 
#define ROS_INTERFACE_H

#include "ros/ros.h"
#include "relay_board/RelayDataMsg.h"
#include "compass/CompassDataMsg.h"
#include "sensor_msgs/NavSatFix.h"
#include "camera_node/CameraDataMsg.h"

#include <iostream>
#include <string>
#include <locale>

void startROS(int argc, char **argv) {
    //initilize ROS and the Node
    ros::init(argc, argv, "topic_logger");
}

void endROS() {
    //Shut down ROS
    ros::shutdown();
}

//this class should be abstracted so all of our nodes can use it instead of copying it for each node
class ROSInterface {
    
    ros::NodeHandle InterfaceHandle;
public:
    ros::Subscriber relay_sub;
    ros::Subscriber compass_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber camera_sub;

public:
    
    ROSInterface(void (*relay_cb)(const relay_board::RelayDataMsg&),
            void (*compass_cb)(const compass::CompassDataMsg&),
            void (*gps_cb)(const sensor_msgs::NavSatFix&),
            void (*camera_cb)(const camera_node::CameraDataMsg&)) {
        
        // Subscribers, with callbacks
        relay_sub = InterfaceHandle.subscribe("RelayData", 1, relay_cb);
        compass_sub = InterfaceHandle.subscribe("CompassData", 1, compass_cb);
        gps_sub = InterfaceHandle.subscribe("GPSData", 1, gps_cb);
        camera_sub = InterfaceHandle.subscribe("CameraData", 1, camera_cb);
        
    }
    
    //lets us know if we should stop the loop
    bool isNodeRunning() {
        return ros::ok();
    }
    
};

#endif

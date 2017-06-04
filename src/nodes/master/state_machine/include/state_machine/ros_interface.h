// State machine ROS Details
#ifndef ROS_INTERFACE_H 
#define ROS_INTERFACE_H

#include "state_machine/vehicle_data.h"
#include "state_machine/globals.h"

#include "ros/ros.h"
#include "relay_board/RelayCommandMsg.h"
#include "relay_board/RelayDataMsg.h"
#include "compass/CompassDataMsg.h"
#include "camera_node/CameraDataMsg.h"
#include "sensor_msgs/NavSatFix.h"
#include "roboteq_msgs/Command.h"
#include "digipot/DigipotDataMsg.h"
#include "state_machine/MsgsForGUI.h"
#include "master_gui/GUImsg.h"
#include "/home/bruin2/bruin_2_code/src/libraries/roboteq/roboteq_driver/include/roboteq_driver/controller.h"

#include <iostream>
#include <string>
#include <locale>


void startROS(int argc, char **argv){
    //initilize ROS and the Node
    ros::init(argc, argv, "state_machine");
}

void endROS(){
    //Shut down ROS
    ros::shutdown();
}

//Callback function definitions for subscribed ROSTopics


// Callback methods execute on each message receipt,
// load data from messages into vehicle_data
void relay_callback(const relay_board::RelayDataMsg& relayStatusMessage) {
}

void compass_callback(const compass::CompassDataMsg& compassMessage) {
    ROS_DEBUG_STREAM( "State Machine: Compass heading: " << compassMessage.heading << std::endl);
    vehicle_data->position_heading = compassMessage.heading;
}

void camera_callback(const camera_node::CameraDataMsg& cameraMessage) {
    ROS_DEBUG_STREAM("Camera direction: " << cameraMessage.direction << " distance:	 " << cameraMessage.distance << "valid:" << cameraMessage.tracking );
    vehicle_data->follow_direction = cameraMessage.direction;
    vehicle_data->follow_distance = cameraMessage.distance;
    vehicle_data->follow_valid = cameraMessage.tracking;
}

void gps_callback(const sensor_msgs::NavSatFix& gpsMessage) {
    ROS_DEBUG_STREAM( "State Machine: GPS Fix: latitude: " << gpsMessage.latitude << ", longitude: " << gpsMessage.longitude << std::endl);
    vehicle_data->position_latitude = gpsMessage.latitude;
    vehicle_data->position_longitude = gpsMessage.longitude;
}

//TODO: make the GUI adjust which waypoint we go to/ which file we open
void gui_callback(const master_gui::GUImsg& guiMessage) {
    ROS_DEBUG_STREAM("GUI station selected: " << guiMessage.state << std::endl);
    vehicle_data->selected_station = guiMessage.state;
    //this isn't necessary but I thought a bool was cleaner than using a string and less prone to error
    if (guiMessage.goToNextState == "True") {
        vehicle_data->goto_button_pressed = true;
    } else {
        vehicle_data-> goto_button_pressed = false;
    }
}



//this class should be abstracted so all of our nodes can use it instead of copying it for each node
class ROSInterface {

    ros::NodeHandle InterfaceHandle;
public:
    ros::Publisher relay_pub;
    ros::Publisher steer_pub;
    ros::Publisher brake_pub;
    ros::Publisher speed_pub;
    ros::Publisher state_pub;
    ros::Subscriber relay_sub;
    ros::Subscriber compass_sub;
    ros::Subscriber camera_sub;
    ros::Subscriber gps_sub;
    ros::Subscriber gui_sub;

  public:

    ROSInterface() {

        
        // Publishers        
        relay_pub   = InterfaceHandle.advertise<relay_board::RelayCommandMsg>("RelayControl", 1000);
        steer_pub   = InterfaceHandle.advertise<roboteq_msgs::Command>("steer/cmd", 1000);
        brake_pub   = InterfaceHandle.advertise<roboteq_msgs::Command>("brake/cmd", 1000);
        speed_pub   = InterfaceHandle.advertise<digipot::DigipotDataMsg>("/digipot/cmd", 1000);
        state_pub   = InterfaceHandle.advertise<state_machine::MsgsForGUI>("CurrentState", 1000);

        // Subscribers, with callbacks
	relay_sub   = InterfaceHandle.subscribe("RelayData", 1, relay_callback);
	compass_sub = InterfaceHandle.subscribe("CompassData", 1, compass_callback);
	camera_sub  = InterfaceHandle.subscribe("CameraData", 1, camera_callback);
        gps_sub     = InterfaceHandle.subscribe("GPSData", 1, gps_callback);
        gui_sub     = InterfaceHandle.subscribe("GUIData", 1, gui_callback);

    }

    //lets us know if we should stop the loop
    bool isNodeRunning(){
        return ros::ok();
    }


};




#endif

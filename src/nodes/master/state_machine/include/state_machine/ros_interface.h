// State machine ROS Details
#ifndef ROS_INTERFACE_H 
#define ROS_INTERFACE_H

#include "state_machine/vehicle_data.h"

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

    ROSInterface(
      void (*relay_cb)(const relay_board::RelayDataMsg&),
      void (*compass_cb)(const compass::CompassDataMsg&),
      void (*camera_cb)(const camera_node::CameraDataMsg&),
      void (*gps_cb)(const sensor_msgs::NavSatFix&),
      void (*gui_cb)(const master_gui::GUImsg&)
    ) {

        
        // Publishers        
        relay_pub   = InterfaceHandle.advertise<relay_board::RelayCommandMsg>("RelayControl", 1000);
        steer_pub   = InterfaceHandle.advertise<roboteq_msgs::Command>("steer/cmd", 1000);
        brake_pub   = InterfaceHandle.advertise<roboteq_msgs::Command>("brake/cmd", 1000);
        speed_pub   = InterfaceHandle.advertise<digipot::DigipotDataMsg>("/digipot/cmd", 1000);
        state_pub   = InterfaceHandle.advertise<state_machine::MsgsForGUI>("CurrentState", 1000);

        // Subscribers, with callbacks
	relay_sub   = InterfaceHandle.subscribe("RelayData", 1, relay_cb);
	compass_sub = InterfaceHandle.subscribe("CompassData", 1, compass_cb);
	camera_sub  = InterfaceHandle.subscribe("CameraData", 1, camera_cb);
        gps_sub     = InterfaceHandle.subscribe("GPSData", 1, gps_cb);
        gui_sub     = InterfaceHandle.subscribe("GUIData", 1, gui_cb);

    }

    //lets us know if we should stop the loop
    bool isNodeRunning(){
        return ros::ok();
    }


};




#endif

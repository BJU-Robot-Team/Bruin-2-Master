// State machine ROS Details
#ifndef ROS_INTERFACE_H 
#define ROS_INTERFACE_H

#include "state_machine/vehicle_data.h"

#include "ros/ros.h"
#include "relay_board/RelayCommandMsg.h"
#include "relay_board/RelayDataMsg.h"
#include "compass/CompassDataMsg.h"
#include "camera_node/CameraDataMsg.h"
#include "roboteq_msgs/Command.h"
#include "digipot/DigipotDataMsg.h"
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
    ros::Subscriber relay_sub;
    ros::Subscriber compass_sub;
    ros::Subscriber camera_sub;


  public:

    ROSInterface(
      void (*relay_cb)(const relay_board::RelayDataMsg&),
      void (*compass_cb)(const compass::CompassDataMsg&),
      void (*camera_cb)(const camera_node::CameraDataMsg&)
    ) {

        
        // Publishers        
        relay_pub   = InterfaceHandle.advertise<relay_board::RelayCommandMsg>("RelayControl", 1000);
        steer_pub   = InterfaceHandle.advertise<roboteq_msgs::Command>("steer/cmd", 1000);
        brake_pub   = InterfaceHandle.advertise<roboteq_msgs::Command>("brake/cmd", 1000);
        speed_pub   = InterfaceHandle.advertise<digipot::DigipotDataMsg>("/digipot/cmd", 1000);

        // Subscribers, with callbacks
	relay_sub   = InterfaceHandle.subscribe("RelayData", 1, relay_cb);
	compass_sub = InterfaceHandle.subscribe("CompassData", 1, compass_cb);
	camera_sub  = InterfaceHandle.subscribe("CameraData", 1, camera_cb);

    }

    //lets us know if we should stop the loop
    bool isNodeRunning(){
        return ros::ok();
    }

    //poll data from listeners
    void pollMessages(VehicleData* vehicle_data) {

	// This program should use Subscribe rather than polling waitforMessage !?!?!?!?

        //get relay data
        auto relay_msg = 
            ros::topic::waitForMessage<relay_board::RelayDataMsg>("RelayData", InterfaceHandle, ros::Duration(0.1));

        if (relay_msg != NULL) {

            std::cout << relay_msg->device_type << " - #: " << relay_msg->device_number << "; State: " <<  relay_msg->state << std::endl;
            if ( relay_msg->device_type == "relay" ) {
                vehicle_data->relay_states[relay_msg->device_number].device_state = relay_msg->state;
            } else if ( relay_msg->device_type == "gpio" ) {
                vehicle_data->gpio_states[relay_msg->device_number].device_state = relay_msg->state;
            } else {
                //print debug
            }

        } else {
            //std::cout << "no relay message" << std::endl;
        }

        //get compass data
        auto compass_msg = 
            ros::topic::waitForMessage<compass::CompassDataMsg>("CompassData", InterfaceHandle, ros::Duration(0.1));

        if (compass_msg != NULL) {

            ROS_DEBUG_STREAM( "State Machine: Compass heading: " << compass_msg->heading << std::endl);
            vehicle_data->position_heading = compass_msg->heading;
            

        } else {
            //std::cout << "no compass message" << std::endl;
        }


	//get camera data
        auto camera_msg = 
            ros::topic::waitForMessage<camera_node::CameraDataMsg>("CameraData", InterfaceHandle, ros::Duration(1));
       if (camera_msg != NULL) {

            ROS_INFO_STREAM("Camera direction: " << camera_msg->direction << " distance:	 " << camera_msg->distance << "valid:" << camera_msg->tracking );
            vehicle_data->follow_direction = camera_msg->direction;
            vehicle_data->follow_distance = camera_msg->distance;
            vehicle_data->follow_valid = camera_msg->tracking;

        } else {
            ROS_INFO_STREAM("no camera message" );
        }

        //updates ROS (if this isn't here ROS doesn't know this node is subcribed)
        ros::spinOnce();

    }

};




#endif

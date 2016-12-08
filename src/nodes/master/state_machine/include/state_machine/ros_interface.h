#ifndef ROS_INTERFACE_H 
#define ROS_INTERFACE_H

#include "state_machine/vehicle_data.h"

#include "ros/ros.h"
#include "relay_board/RelayCommandMsg.h"
#include "relay_board/RelayDataMsg.h"
#include "compass/CompassDataMsg.h"

#include <iostream>
#include <string>
#include <locale>


void startROS(int argc, char **argv){
    //initilize ROS and the Node
    ros::init(argc, argv, "state_machine");
}

//this class should be abstracted so all of our nodes can use it instead of copying it for each node
class ROSInterface {

    ros::NodeHandle InterfaceHandle;
    ros::Publisher relay_pub;


  public:

    ROSInterface() {

        
        
        relay_pub = InterfaceHandle.advertise<relay_board::RelayCommandMsg>("RelayControl", 1000);

    }

    //lets us know if we should stop the loop
    bool isNodeRunning(){
        return ros::ok();
    }

    //poll data from listeners
    void pollMessages(VehicleData* vehicle_data) {

        //get relay data
        auto relay_msg = 
            ros::topic::waitForMessage<relay_board::RelayDataMsg>("RelayData", InterfaceHandle, ros::Duration(.01));

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
            std::cout << "no message" << std::endl;
        }

        //get compass data
        auto compass_msg = 
            ros::topic::waitForMessage<compass::CompassDataMsg>("CompassData", InterfaceHandle, ros::Duration(.01));

        if (compass_msg != NULL) {

            std::cout << "Compass heading: " << compass_msg->heading << std::endl;
            vehicle_data->position_heading = compass_msg->heading;
            

        } else {
            std::cout << "no message" << std::endl;
        }


        //updates ROS (if this isn't here ROS doesn't know this node is subcribed)
        ros::spinOnce();

    }

    //push data to listeners
    void publishMessages(std::string command) {

        if (ros::ok()) {
            relay_board::RelayCommandMsg message;

            //sending a command for a relay
            message.device_type = "relay";

            //setting the device the cammand is for
            message.device_number = 0;

            //setting which command to do
            message.command = command;

            //Publish message
            relay_pub.publish(message);

            ros::spinOnce();
            
        } else {
            //TODO: handle debug here. ROS has gone down for some reason.
        }

    }

};




#endif

#ifndef ROS_INTERFACE_H 
#define ROS_INTERFACE_H

#include "ros/ros.h"
#include "relay_board/RelayCommandMsg.h"
#include "relay_board/RelayDataMsg.h"


void startROS(int argc, char **argv){
    //initilize ROS and the Node
    ros::init(argc, argv, "relay_board");
}

//this class should be abstracted so all of our nodes can use it instead of copying it for each node
class ROSInterface {

    ros::NodeHandle InterfaceHandle;
    ros::Publisher state_machine_pub;

  public:

    ROSInterface() {

        
        //for sending data to the state machine
        state_machine_pub = InterfaceHandle.advertise<relay_board::RelayDataMsg>("RelayData", 1000);
        

    }

    //lets us know if we should stop the loop
    bool isNodeRunning(){
        return ros::ok();
    }

    //poll data from listeners
    bool pollMessages(std::string& device_type, int& device_num, std::string& command) {

         auto relay_msg = 
            ros::topic::waitForMessage<relay_board::RelayCommandMsg>("RelayControl", InterfaceHandle, ros::Duration(.01));

        //updates ROS (if this isn't here ROS doesn't know this node is subcribed)
        ros::spinOnce();

        if (relay_msg != NULL) {

            device_type = relay_msg->device_type;
            device_num = relay_msg->device_number;
            command = relay_msg->command;

            return true;

        } else {
            // std::cout << "no message" << std::endl;
            return false;
        }

        
    }

    //push data to listeners
    void publishMessages(std::string type, int number, std::string state) {

        if (ros::ok()) {
            relay_board::RelayDataMsg message;

            //The type of device (relay or GPIO) that the data is from
            message.device_type = type;

            //the device number the data is from
            message.device_number = number;

            //the current state of the specified device
            message.state = state;

            //Publish message
            state_machine_pub.publish(message);

            ros::spinOnce();
            
        } else {
            //TODO: handle debug here. ROS has gone down for some reason.
        }

    }

};




#endif

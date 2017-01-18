// Digipot ROS interface details
#ifndef ROS_INTERFACE_H 
#define ROS_INTERFACE_H

#include "ros/ros.h"
#include "digipot/DigipotDataMsg.h"
#include "digipot/DigipotStatusMsg.h"

#include "digipot/digipot_data.h"


void startROS(int argc, char **argv){
    //initilize ROS and the Node
    ros::init(argc, argv, "digipot");
}



//this class should be abstracted so all of our nodes can use it instead of copying it for each node
class ROSInterface {

    ros::NodeHandle InterfaceHandle;
    ros::Publisher digipot_pub;
    ros::Subscriber sub_command;

  public:

    ROSInterface(  void (*cb)(const digipot::DigipotDataMsg&) ) {

        
        //for sending data to the state machine
        digipot_pub = InterfaceHandle.advertise<digipot::DigipotStatusMsg>("digipot/status", 1000);
        sub_command = InterfaceHandle.subscribe("digipot/cmd", 1, cb);

    }

    //lets us know if we should stop the loop
    bool isNodeRunning(){
        return ros::ok();
    }




    //push data to listeners
    void publishMessages(Digipot_Status& data) {

        if (ros::ok()) {
            digipot::DigipotStatusMsg message;

            //heading of the compass
            message.speed = data.speed;

            //Publish message
            digipot_pub.publish(message);

            ros::spinOnce();
            
        } else {
            //TODO: handle debug here. ROS has gone down for some reason.
        }

    }

};




#endif

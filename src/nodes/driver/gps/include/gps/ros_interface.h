#ifndef ROS_INTERFACE_H 
#define ROS_INTERFACE_H

#include "ros/ros.h"
#include "gps/GpsDataMsg.h"

#include "gps/gps_data.h"


void startROS(int argc, char **argv){
    //initilize ROS and the Node
    ros::init(argc, argv, "gps");
}


//this class should be abstracted so all of our nodes can use it instead of copying it for each node
class ROSInterface {

    ros::NodeHandle InterfaceHandle;
    ros::Publisher state_machine_pub;

  public:

    ROSInterface() {

        
        //for sending data to the state machine
        state_machine_pub = InterfaceHandle.advertise<gps::GpsDataMsg>("GpsData", 1000);
        

    }

    //lets us know if we should stop the loop
    bool isNodeRunning(){
        return ros::ok();
    }


    //push data to listeners
    void publishMessages(Gps_Data& data) {

        if (ros::ok()) {
            gps::GpsDataMsg message;

            //heading of the compass
            message.heading = data.heading;

            //pitch of the compass
            message.pitch = data.pitch;

            //roll of the compass
            message.roll = data.roll;

            //temperature of the compass
            message.temperature = data.temperature;

            //Publish message
            state_machine_pub.publish(message);

            ros::spinOnce();
            
        } else {
            //TODO: handle debug here. ROS has gone down for some reason.
            // Maybe abstract these device drivers to instead of device specific method names use "device_method" 
        }

    }

};




#endif

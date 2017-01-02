#ifndef ROS_INTERFACE_H 
#define ROS_INTERFACE_H

#include "ros/ros.h"
#include "camera_node/CameraDataMsg.h"


void startROS(int argc, char **argv){
    //initilize ROS and the Node
    ros::init(argc, argv, "camera");
}


//this class should be abstracted so all of our nodes can use it instead of copying it for each node
class ROSInterface {

    ros::NodeHandle InterfaceHandle;
    ros::Publisher state_machine_pub;

  public:

    ROSInterface() {

        
        //for sending data to the state machine
        state_machine_pub = InterfaceHandle.advertise<camera_node::CameraDataMsg>("CameraData", 1000);
        

    }

    //lets us know if we should stop the loop
    bool isNodeRunning(){
        return ros::ok();
    }


    //push data to listeners
    void publishMessages(bool tracking, float direction, float distance) {

        if (ros::ok()) {
            camera_node::CameraDataMsg message;

            //Message has only one boolean field currently
            message.tracking = tracking;
	    message.direction = direction;
            message.distance = distance;

            //Publish message
            state_machine_pub.publish(message);

            ros::spinOnce();
            
        } else {
            //TODO: handle debug here. ROS has gone down for some reason.
        }

    }

};




#endif

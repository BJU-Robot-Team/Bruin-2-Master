#include "../headers/actuators.h"
#include "ros/ros.h"
#include "bruin2_msgs/RelayCommandMsg.h"
#include "bruin2_msgs/RelayDataMsg.h"

// TODO: get correct relay numbers
#define BRAKE_RELAY_NUM 13 // TBD
#define MOTOR_RELAY_SPEED_0 8
#define MOTOR_RELAY_SPEED_1_5 9
#define MOTOR_RELAY_SPEED_5_5 11

// -----------------------------------------------------------------------
// helper functions
// -----------------------------------------------------------------------

ros::Publisher relay_pub;

bool setup = false;

void initRelay() {
    setup = true;
    ros::NodeHandle InterfaceHandle;
    relay_pub = InterfaceHandle.advertise<bruin2_msgs::RelayCommandMsg>("RelayControl", 1);
}

// Get the state of a specified relay.
// sends a message to the relay controller and receives a message back about whether the relay is on
// true means the relay is on
bool getRelayState(int relayNum) {
    if (!setup) {
        initRelay();
    }
    
    
    // send the query
    bruin2_msgs::RelayCommandMsg relayMessage;
    relayMessage.device_type = "relay";
    relayMessage.device_number = relayNum;
    relayMessage.command = "read";
    relay_pub.publish(relayMessage);
    
    // receive the results
    ros::NodeHandle InterfaceHandle;
    auto relay_msg = ros::topic::waitForMessage<bruin2_msgs::RelayDataMsg> ("RelayData", ros::Duration(5));

    //updates ROS (if this isn't here ROS doesn't know this node is subcribed)
    ros::spinOnce();
    
    ros::Rate r(10); // 10 Hz
    while (relay_msg == NULL) {
        relay_msg = ros::topic::waitForMessage<bruin2_msgs::RelayDataMsg> ("RelayData", ros::Duration(5));
        ros::spinOnce();
        r.sleep();
    }
    return relay_msg->state == "on";
}

// Turn the given relay on or off
bool sendRelayMessage(int relayNum, bool on) {
    if (!setup) {
        initRelay();
    }
    bruin2_msgs::RelayCommandMsg relayMessage;
    relayMessage.device_type = "relay";
    relayMessage.device_number = relayNum;
    relayMessage.command = on ? "on" : "off";
    relay_pub.publish(relayMessage);
    
    // return whether the command worked
    return getRelayState(relayNum) == on;
}

// -----------------------------------------------------------------------
// brake commands
// -----------------------------------------------------------------------
bool Brake::set() {
    return sendRelayMessage(BRAKE_RELAY_NUM, true);
}

bool Brake::release() {
    return sendRelayMessage(BRAKE_RELAY_NUM, false);
}

// -----------------------------------------------------------------------
// Motor commands
// -----------------------------------------------------------------------
bool Motor::noSpeed() {
    return sendRelayMessage(MOTOR_RELAY_SPEED_0, true) and
        sendRelayMessage(MOTOR_RELAY_SPEED_1_5, false) and
        sendRelayMessage(MOTOR_RELAY_SPEED_5_5, false);
}

bool Motor::fifthSpeed() {
    return sendRelayMessage(MOTOR_RELAY_SPEED_0, false) and
        sendRelayMessage(MOTOR_RELAY_SPEED_1_5, true) and
        sendRelayMessage(MOTOR_RELAY_SPEED_5_5, false);
}

bool Motor::fullSpeed() {
    return sendRelayMessage(MOTOR_RELAY_SPEED_0, false) and
        sendRelayMessage(MOTOR_RELAY_SPEED_1_5, false) and
        sendRelayMessage(MOTOR_RELAY_SPEED_5_5, true);
}

Speed Motor::getSpeed() {
    if (getRelayState(MOTOR_RELAY_SPEED_0)) { return NONE; }
    if (getRelayState(MOTOR_RELAY_SPEED_1_5)) { return FIFTH; }
    if (getRelayState(MOTOR_RELAY_SPEED_5_5)) { return FULL; }
    return NONE; // SOMETHING's really wrong
}



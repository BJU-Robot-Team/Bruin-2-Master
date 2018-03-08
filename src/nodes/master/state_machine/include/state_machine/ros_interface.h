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
#include "../../../../libraries/roboteq/roboteq_driver/include/roboteq_driver/controller.h"

#include <iostream>
#include <string>
#include <locale>

void startROS(int argc, char **argv) {
    //initilize ROS and the Node
    ros::init(argc, argv, "state_machine");
}

void endROS() {
    //Shut down ROS
    ros::shutdown();
}

//Callback function definitions for subscribed ROSTopics

// Callback methods execute on each message receipt,
// load data from messages into vehicle_data
void relay_callback(const relay_board::RelayDataMsg& relayStatusMessage) {
}

void compass_callback(const compass::CompassDataMsg& compassMessage) {
    ROS_DEBUG_STREAM(
            "State Machine: Compass heading: " << compassMessage.heading
                    << std::endl);
    vehicle_data->position_heading = compassMessage.heading;
}

void camera_callback(const camera_node::CameraDataMsg& cameraMessage) {
    ROS_DEBUG_STREAM(
            "Camera direction: " << cameraMessage.direction << " distance:	 "
                    << cameraMessage.distance << "valid:"
                    << cameraMessage.tracking);
    vehicle_data->follow_direction = cameraMessage.direction;
    vehicle_data->follow_distance = cameraMessage.distance;
    vehicle_data->follow_valid = cameraMessage.tracking;
}

void gps_callback(const sensor_msgs::NavSatFix& gpsMessage) {
    ROS_DEBUG_STREAM(
            "State Machine: GPS Fix: latitude: " << gpsMessage.latitude
                    << ", longitude: " << gpsMessage.longitude << std::endl);
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
        vehicle_data->goto_button_pressed = false;
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

    roboteq_msgs::Command steerMessage;
    roboteq_msgs::Command brakeMessage;
    digipot::DigipotDataMsg speedMessage;
    relay_board::RelayCommandMsg relayMessage;
    relay_board::RelayDataMsg relayDataMessage;
    state_machine::MsgsForGUI StateOfRobotMessage;

public:
    
    ROSInterface() {
        
        // Publishers        
        relay_pub = InterfaceHandle.advertise < relay_board::RelayCommandMsg
                > ("RelayControl", 1000);
        steer_pub = InterfaceHandle.advertise < roboteq_msgs::Command
                > ("steer/cmd", 1000);
        brake_pub = InterfaceHandle.advertise < roboteq_msgs::Command
                > ("brake/cmd", 1000);
        speed_pub = InterfaceHandle.advertise < digipot::DigipotDataMsg
                > ("/digipot/cmd", 1000);
        state_pub = InterfaceHandle.advertise < state_machine::MsgsForGUI
                > ("CurrentState", 1000);
        
        // Subscribers, with callbacks
        relay_sub = InterfaceHandle.subscribe("RelayData", 1, relay_callback);
        compass_sub = InterfaceHandle.subscribe("CompassData", 1,
                compass_callback);
        camera_sub = InterfaceHandle.subscribe("CameraData", 1,
                camera_callback);
        gps_sub = InterfaceHandle.subscribe("GPSData", 1, gps_callback);
        gui_sub = InterfaceHandle.subscribe("GUIData", 1, gui_callback);
        
    }
    
    //lets us know if we should stop the loop
    bool isNodeRunning() {
        return ros::ok();
    }
    
    void publishAllMessages(std::string current_state) {
        
        //populate mesages before publishing them
        
        relayMessage.device_type = "relay";
        relayMessage.device_number = 0;
        relayMessage.command = "writeall";
        
        //call warning light method
        adjustLight(vehicle_data->turn_off_light, vehicle_data->light_count);
        
        if (vehicle_data->speed_cmd < 0.1) {
            // Faking pot with relays; this is speed 0
            relayMessage.mask = 0x0000 | FORWARD_RELAY;
            
        } else if (vehicle_data->speed_cmd <= 1) {
            // second fixed speed, FORWARD
            relayMessage.mask = 0x0300 | START_RELAY | FORWARD_RELAY;
            
        } else if (vehicle_data->speed_cmd <= 2) {
            // third fixed speed, FORWARD
            relayMessage.mask = 0x0500 | START_RELAY | FORWARD_RELAY;
            
        } else if (vehicle_data->speed_cmd <= 3) {
            // fourth fixed speed, FORWARD
            relayMessage.mask = 0x0900 | START_RELAY | FORWARD_RELAY;
            
        } else {
            // fifth fixed speed, FORWARD
            relayMessage.mask = 0x1100 | START_RELAY | FORWARD_RELAY;
            
        }
        
        // Add the state of the flashing light
        if (!vehicle_data->turn_off_light) {
            relayMessage.mask = relayMessage.mask | FLASHING_LIGHT;
        }
        
        steerMessage.mode = 1; // 1=MODE_POSITION, 0=MODE_SPEED
        brakeMessage.mode = 1;
        
        steerMessage.setpoint = STEER_OFFSET + vehicle_data->steer_cmd * 1.5; // deliberately oversteer
                
        if (speedMessage.speed < (vehicle_data->speed_cmd - 1)) {
            // Don't drop speed suddenly from high speed to zero
            speedMessage.speed = vehicle_data->speed_cmd - 1;
        } else {
            speedMessage.speed = vehicle_data->speed_cmd;
        }
        
        brakeMessage.setpoint = vehicle_data->brake_cmd;
        
        //publish the messages we got in the interface
        StateOfRobotMessage.currentState = current_state;
        
        state_pub.publish(StateOfRobotMessage);
        steer_pub.publish(steerMessage);
        brake_pub.publish(brakeMessage);
        speed_pub.publish(speedMessage);
        relay_pub.publish(relayMessage);
        
    }
    
    //Warning light code
    //TODO: should be able to be invoked by a ROS Timer
    void adjustLight(bool turn_off_light, int light_count) {
        
        //if the turn_off_light is true and the light count is less than ten increment the light count
        if (turn_off_light) {
            if (light_count < 10) { //cycles for light to be on
                light_count = light_count + 1;
                //otherwise, set the command to off, the turn_off_light again to false and count to 0
            } else {
                relayMessage.command = "OFF";
                turn_off_light = false;
                light_count = 0;
            }
            //otherise, if turn_off_light is false
        } else {
            //if light count is less than ten, increment the light count
            if (light_count < 10) { //cycles for light to be off
                light_count = light_count + 1;
                //otherwise, set the command to on, the turn_off_light again to true and count to 0
            } else {
                relayMessage.command = "ON";
                turn_off_light = true;
                light_count = 0;
            }
        }
    }
    
};

#endif

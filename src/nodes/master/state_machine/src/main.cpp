

#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"
#include "state_machine/ros_interface.h"

#include <vector>
#include <string>

//print vehicle data for debug purposes
//TODO: Should be in VehicleData.h but has multiple definitions error and I ran out of time.
void vehicleDataPrint(VehicleData* vehicle_data){
    std::cout << "########################" << std::endl
              << "Vehicle Data Print out"   << std::endl
              << "########################" << std::endl;

    std::cout << "<< Compass Data >>" << std::endl 
              << "Compass Heading: "
              <<  std::to_string(vehicle_data->position_heading) << std::endl;

    std::cout << "<< GPS Data >>" << std::endl 
              << "GPS Latitude: "
              <<  std::to_string(vehicle_data->position_latitude) << std::endl
              << "GPS Longetude: "
              <<  std::to_string(vehicle_data->position_longitude) << std::endl;

    //maybe print out other data like all relay/gpio states
};



int main(int argc, char **argv) {

    //function defined in ros_interface header
    startROS(argc, argv);

    //setup ros interface, state machine, and vehicle data  objects
    ROSInterface ros_interface;
    StateMachine state_machine;
    VehicleData* vehicle_data = new VehicleData();
    
    state_machine.debug_mode = false;
    bool turn_off_light = false;
    int light_count = 0;

    //start main loop
    while (ros_interface.isNodeRunning()) {


        //Warrning light code
        //TODO: this should have it's own place not slapped int the middle of the main loop. But I'm out of time.
        if (turn_off_light) {
            if (light_count < 1000) { //cycles for light to be on
                light_count = light_count+1;
            } else {
                ros_interface.publishMessages("OFF");
                turn_off_light = false;
                light_count = 0;
            }

        } else {

            if (light_count < 1000) { //cycles for light to be off
                light_count = light_count+1;
            } else {
                ros_interface.publishMessages("ON");
                turn_off_light = true;
                light_count = 0;
            }
        }	

        //check all ROS topics and update vehicle data object
        ros_interface.pollMessages(vehicle_data);

        //Run state machine tick
        state_machine.tick(vehicle_data);

        //debug print data
        if (state_machine.debug_mode) {
            vehicleDataPrint(vehicle_data);
        }

        // Publish the command messages
        roboteq_msgs::Command steerMessage;
        roboteq_msgs::Command brakeMessage;
        roboteq_msgs::Command speedMessage;

        steerMessage.mode = 1; // 1=MODE_POSITION, 0=MODE_SPEED	
        brakeMessage.mode = 1;
        speedMessage.mode = 0; // MODE_SPEED

        steerMessage.setpoint = vehicle_data->steer_cmd;
        speedMessage.setpoint = vehicle_data->speed_cmd;
        brakeMessage.setpoint = vehicle_data->brake_cmd;

        ros_interface.steer_pub.publish(steerMessage);
        ros_interface.brake_pub.publish(brakeMessage);
        ros_interface.speed_pub.publish(speedMessage);

        //check if we are finished
    }
}

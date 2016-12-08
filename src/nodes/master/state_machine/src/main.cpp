

#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"
#include "state_machine/ros_interface.h"

#include <vector>
#include <string>

int main(int argc, char **argv) {

    //function defined in ros_interface header
    startROS(argc, argv);

    //setup ros interface, state machine, and vehicle data  objects
    ROSInterface ros_interface;
    StateMachine state_machine;
    VehicleData* vehicle_data = new VehicleData();
    
    state_machine.debug_mode = true;
    bool turn_off = false;
    //start main loop
    while (ros_interface.isNodeRunning()) {

        //check all ROS topics and update vehicle data object
        ros_interface.pollMessages(vehicle_data);

        //debug
        if (turn_off) {
            ros_interface.publishMessages("OFF");
            turn_off = false;
        } else {
            ros_interface.publishMessages("ON");
            turn_off = true;
        }

        //Run state machine tick
        state_machine.tick(vehicle_data);

        //check if we are finished
    }
}

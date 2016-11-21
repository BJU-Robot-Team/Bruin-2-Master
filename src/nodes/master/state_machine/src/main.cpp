

#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"


#include <vector>
#include <string>

int main(void) {


    //setup ros interface, state machine, and vehicle data  objects
    //ROSInterface ros_interface();
    StateMachine state_machine;
    VehicleData* vehicle_data = new VehicleData();
    
    state_machine.debug_mode = true;

    //start main loop
    bool running = true;
    while (running) {

        //check all ROS topics and update vehicle data object
        //ros_interface.pollTopics(vehicle_data);

        //Run state machine tick
        state_machine.tick(vehicle_data);

        //check if we are finished
    }
}

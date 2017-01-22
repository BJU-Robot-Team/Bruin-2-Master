#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void ShutdownState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {

    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) { return; } //end running imediatly 
    }

    std::cout << "State Shutdown." << std::endl;
    vehicle_data->shutdown = true;

}




bool ShutdownState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;


    return queryUserForTransition("SHUTDOWN", state_machine, line_desc,  event_lines);
}

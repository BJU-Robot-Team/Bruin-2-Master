#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void WaitForCommandState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {

    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) { return; } //end running imediatly 
    }


    //check if destination station has been set
        //if so trigger state transition to initial obstacle detect

}




bool WaitForCommandState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;

    line_desc.push_back("SHUTDOWN_REQUESTED");
    event_lines.push_back(SHUTDOWN_REQUESTED);

    line_desc.push_back("STATION_REQUESTED");
    event_lines.push_back(STATION_REQUESTED);

    return queryUserForTransition(state_machine, line_desc,  event_lines);
}

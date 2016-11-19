#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void InitilizeState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {

    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) { return; } //end running imediatly 
    }
        

    //check if waypoint map is loaded
        //if not load it


    //check GPS accuracy
        //if GPS is good calculate corrections needed for the compass

    //Check if all ROS modules have sent in a running ping


    //if all checks above come out true send state machine a transition event
    //state_machine->internalEvent(INITIALIZE_FINISHED)
}


bool InitilizeState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;

    line_desc.push_back("INITIALIZE_FINISHED");
    event_lines.push_back(INITIALIZE_FINISHED);

    return queryUserForTransition(state_machine, line_desc,  event_lines);
}

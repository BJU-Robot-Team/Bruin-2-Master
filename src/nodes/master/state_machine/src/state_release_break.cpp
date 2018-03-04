#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void ReleaseBrakeState::tick(StateMachine* state_machine,
        VehicleData* vehicle_data) {
    
    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if (!debugState(state_machine)) {
            return;
        } //end running imediatly 
    }
    
    //if break is not releaseing send break release command
    
    //check if break has released
    //if it has, send state transition to check destination
    
}

bool ReleaseBrakeState::debugState(StateMachine* state_machine) {
    
    std::vector < std::string > line_desc;
    std::vector<VehicleEvents> event_lines;
    
    line_desc.push_back("BREAK_RELEASED");
    event_lines.push_back(BREAK_RELEASED);
    
    return queryUserForTransition("RELEASE_BRAKE", state_machine, line_desc,
            event_lines);
}

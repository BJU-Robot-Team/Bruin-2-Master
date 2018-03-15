#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void CheckStopSignState::tick(StateMachine* state_machine,
        VehicleData* vehicle_data) {
    
    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if (!debugState(state_machine)) {
            return;
        } //end running immediately 
    }
    
    //check if we are at a stop sign
    //yes then transition to performing taxi stop
    
    //no then transition to drive path
}

bool CheckStopSignState::debugState(StateMachine* state_machine) {
    std::vector < std::string > line_desc;
    std::vector<VehicleEvents> event_lines;
    
    line_desc.push_back("NO_STOP_SIGN");
    event_lines.push_back(NO_STOP_SIGN);
    
    line_desc.push_back("STOP_SIGN");
    event_lines.push_back(STOP_SIGN);
    
    return queryUserForTransition("CHECK_STOP_SIGN", state_machine, line_desc,
            event_lines);
}

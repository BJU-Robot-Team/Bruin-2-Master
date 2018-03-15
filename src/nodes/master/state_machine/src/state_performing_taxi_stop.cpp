#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void PerformingTaxiStopState::tick(StateMachine* state_machine,
        VehicleData* vehicle_data) {
    
    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if (!debugState(state_machine)) {
            return;
        } //end running immediately 
    }
    
    //check if timer is setup
    //if not set it up
    
    //if it check if timer is 0
    //if timer at 0 transition to Drive path
    
    //if timer > 0 end tick
    
}

bool PerformingTaxiStopState::debugState(StateMachine* state_machine) {
    
    std::vector < std::string > line_desc;
    std::vector<VehicleEvents> event_lines;
    
    line_desc.push_back("TAXI_STOP_FINISHED");
    event_lines.push_back(TAXI_STOP_FINISHED);
    
    return queryUserForTransition("PERFORMING_TAXI_STOP", state_machine,
            line_desc, event_lines);
}

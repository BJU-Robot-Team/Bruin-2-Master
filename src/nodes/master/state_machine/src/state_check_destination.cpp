#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void CheckDestinationState::tick(StateMachine* state_machine,
        VehicleData* vehicle_data) {
    
    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if (!debugState(state_machine)) {
            return;
        } //end running imediatly 
    }
    
    //compare current GPS location with end station GPS waypoint and see if 
    //    they are within the waypoint's tolerance
    //if within an acceptable distance from station sent state transition to Park
    
    //if not within an acceptable distance send state transition to check stop sign
    
}

bool CheckDestinationState::debugState(StateMachine* state_machine) {
    
    std::vector < std::string > line_desc;
    std::vector<VehicleEvents> event_lines;
    
    line_desc.push_back("NOT_AT_DESTINATION");
    event_lines.push_back(NOT_AT_DESTINATION);
    
    line_desc.push_back("AT_DESTINATION");
    event_lines.push_back(AT_DESTINATION);
    
    return queryUserForTransition("DETECT_STATION", state_machine, line_desc,
            event_lines);
}

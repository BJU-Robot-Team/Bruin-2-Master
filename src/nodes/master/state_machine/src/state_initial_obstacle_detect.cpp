#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void InitialObstacleDetectState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {

    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) { return; } //end running imediatly 
    }


    //check for Obstacles
        //if no obstacles repeat x times 
            //trigger transition event to release break

        //if there are obstacles repeat x times
            //send message to GUI and trigger state transition to wait for command

}


bool InitialObstacleDetectState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;

    line_desc.push_back("NO INITIAL OBSTICAL DETECTED");
    event_lines.push_back(NO_INITIAL_OBSTICAL_DETECTED);

    line_desc.push_back("INITIAL OBSTICAL DETECTED");
    event_lines.push_back(INITIAL_OBSTICAL_DETECTED);

    return queryUserForTransition(state_machine, line_desc,  event_lines);
}

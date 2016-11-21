#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void DrivePathState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {

    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) { return; } //end running imediatly 
    }

    //run pathfinder
        //no open path
            //stop vehicle


        //open path availible
            //open path requires lane change
                //run lane change sub-routine 
            //open path streight
            
            //open path curve

}



bool DrivePathState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;

    line_desc.push_back("PATH FINISHED");
    event_lines.push_back(PATH_FINISHED);

    return queryUserForTransition("DRIVE_PATH", state_machine, line_desc,  event_lines);
}

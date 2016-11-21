#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"

#include <string>

void DetectStationState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {

    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) { return; } //end running imediatly 
    }

    //Compare GPS location to locations on the waypoint map
        //if locations match up within tolerences, set the vehicle's start station
        //trigger state transition since we have detected the station

        //if no station matchs try again (end this tick)
            //after x retries send gui message that a station could not be found
            //    provide instructions to drive to the nearest station

}



bool DetectStationState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;

    line_desc.push_back("STATION_DETECTED");
    event_lines.push_back(STATION_DETECTED);

    return queryUserForTransition("DETECT_STATION", state_machine, line_desc,  event_lines);
}

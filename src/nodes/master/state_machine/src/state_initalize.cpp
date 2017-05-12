#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"
#include "state_machine/waypoint_map.h"

#include <string>
#include <iostream>

void InitilizeState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {



    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) { return; } //end running imediatly
    }

    bool waypoint_map_loaded = true;
    bool gps_accurate = true; //TODO set to false once implemented below.
    bool ros_moduals_active = true; //TODO set to false once implemented below.
    //check if waypoint map is loaded

    if (vehicle_data->waypoint_map == NULL) {

        //load waypoint map
        //vehicle_data->waypoint_map = new WaypointMap("data/maps/testmap");
        //TODO need to do some error checking.

        //map is loaded so we set bool to true
        waypoint_map_loaded = true;
    }



    //check GPS accuracy
        //if GPS is good calculate corrections needed for the compass

    //Check if all ROS modules have sent in a running ping


    //if all checks above come out true send state machine a transition event
    if (waypoint_map_loaded and gps_accurate and ros_moduals_active ) {
        state_machine->internalEvent(INITIALIZE_FINISHED);
    }
}


bool InitilizeState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;

    line_desc.push_back("INITIALIZE_FINISHED");
    event_lines.push_back(INITIALIZE_FINISHED);
    line_desc.push_back("ENTER_FOLLOW");
    event_lines.push_back(ENTER_FOLLOW);

    return queryUserForTransition("INITIALIZE", state_machine, line_desc,  event_lines);
}

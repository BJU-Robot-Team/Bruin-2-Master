// State Joystick - drive in simulated joystick mode with keyboard
// generate /steer/cmd, /brake/cmd, and /speed/cmd messages
// V1.0 Bill Lovegrove
#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"
#include "roboteq_msgs/Command.h"
#include "/home/bruin2/bruin_2_code/src/libraries/roboteq/roboteq_driver/include/roboteq_driver/controller.h"

using namespace std;

#include <string>
#include <cmath>

void JoystickState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {

    char c;

    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        ROS_DEBUG("Joystick Debug mode");
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) {
           ROS_DEBUG("Debug state false"); 
           return; 
        } //end running immediately 
    }

    std::cout << "Go, Stop, Follow, Quit" << std::endl;    
    //Wait for a keypress.
    std::cin >> c;

    switch(c) {
    case 'g':
    case 'G': {
        vehicle_data->speed_cmd = 1;
        ROS_DEBUG_STREAM( "Joystick state: Go");
        break;
    }
    case 's':
    case 'S': {
        vehicle_data->speed_cmd = 0;
        ROS_DEBUG_STREAM( "Joystick state: Stop");
        break;
    }
    case 'f':
    case 'F': {
        ROS_DEBUG_STREAM( "Joystick state: Follow");
        state_machine->internalEvent(ENTER_FOLLOW);
        break;
    }
    default:
        vehicle_data->speed_cmd = 0;
        ROS_DEBUG_STREAM( "Joystick state: Quit");
        break;
    }
}



bool JoystickState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;

    line_desc.push_back("SHUTDOWN_REQUESTED");
    event_lines.push_back(SHUTDOWN_REQUESTED);
    line_desc.push_back("ENTER_FOLLOW");
    event_lines.push_back(ENTER_FOLLOW);

    return queryUserForTransition("JOYSTICK", state_machine, line_desc,  event_lines);
}

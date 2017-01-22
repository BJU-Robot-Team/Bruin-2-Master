// State FOLLOW - steer and drive the robot to follow a green/blue target based on /CameraData messages
// generate /steer/cmd, /brake/cmd, and /speed/cmd messages
// V1.1 Bill Lovegrove
#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"
#include "roboteq_msgs/Command.h"
#include "/home/bruin2/bruin_2_code/src/libraries/roboteq/roboteq_driver/include/roboteq_driver/controller.h"

using namespace std;

#define FOLLOW_MIN 2   // min follow distance in meters
#define FOLLOW_GAIN 1  // m/s per m of distance beyond minimum
#define WHEELBASE 2     // wheelbase in meters

#define RAD_TO_POS    1273   // (180/Pi)*1000/45 (max counts / max degrees)

#include <string>
#include <cmath>

void FollowState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {

    char c;

    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        ROS_DEBUG("follow Debug mode");
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) {
           ROS_DEBUG("Debug state false"); 
           return; 
        } //end running immediately 
    }
    

    if ( vehicle_data->follow_valid ) {
      if ( vehicle_data->follow_distance < FOLLOW_MIN ) {
        // Too close, speed=0, brake on
	ROS_DEBUG_STREAM( "Camera: Too close: Brake on. Speed 0");
        vehicle_data->brake_cmd = 1000;
        vehicle_data->speed_cmd = 0;
      }
      else {
        // Calculate follow speed
        vehicle_data->brake_cmd = 0;
        vehicle_data->speed_cmd = (vehicle_data->follow_distance - FOLLOW_MIN)*FOLLOW_GAIN;
	ROS_DEBUG_STREAM( "Camera: Command speed " << vehicle_data->speed_cmd );
      }
      // Either way, calculate steer direction
      vehicle_data->steer_cmd = RAD_TO_POS * atan( 2 * sin(vehicle_data->follow_direction) / (WHEELBASE*vehicle_data->follow_distance) );
      ROS_DEBUG_STREAM( "Camera: Command steer " << vehicle_data->steer_cmd );
    }
    else {
      // Not tracking, speed=0, steer=0, brake on
        vehicle_data->brake_cmd = 1;
        vehicle_data->speed_cmd = 0;
      ROS_DEBUG_STREAM( "Camera: not valid tracking");
    }
  
    //Check for a keypress (Can't wait here or state machine will stop)
    c = vehicle_data->char_input;
    vehicle_data->char_input = 0;


    switch(c) {
    case 0:
        break;
    case 'j':
    case 'J': {
        ROS_DEBUG_STREAM( "Follow_command: joystick");
        state_machine->internalEvent(ENTER_JOYSTICK);
        break;
    }
    case 'q':
    case 'Q': {
        ROS_DEBUG_STREAM( "Follow Command: Quit");
        state_machine->internalEvent(SHUTDOWN_REQUESTED);
        break;
    }
    default:
        break;
    }

}



bool FollowState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;

    line_desc.push_back("SHUTDOWN_REQUESTED");
    event_lines.push_back(SHUTDOWN_REQUESTED);

    return queryUserForTransition("FOLLOW", state_machine, line_desc,  event_lines);
}

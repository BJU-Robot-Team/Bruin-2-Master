#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"
#include "state_machine/waypoint_math.h"
#include <string>

#include "state_machine/state_interfaces.h"
#include "state_machine/debug_utils.h"
#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"
#include "roboteq_msgs/Command.h"
#include "/home/bruin2/bruin_2_code/src/libraries/roboteq/roboteq_driver/include/roboteq_driver/controller.h"
using namespace std;


void DrivePathState::tick(StateMachine* state_machine, VehicleData* vehicle_data) {

    //call the state's debug function if we are in debug mode.
    if (state_machine->debug_mode) {
        //run the state's debug function if it returns true continue normal code
        if(!debugState(state_machine)) { return; } //end running immediatly 
    }

	

	//Read vehicle state pose
	//cout<<"pose: " << vehicle_data->position_latitude << vehicle_data->position_longitude << vehicle_data->position_heading;

	
	// load vehicle location, loaded from vehicle pose data
	double x1 = waypoint_math.computeLocalX(vehicle_data->position_longitude);
	double y1 = waypoint_math.computeLocalY(vehicle_data->position_latitude);

	//proportional gain for steering angle, chosen through experimentation
	double k = 0.5; //adjust as needed
	
	// load Vehicle angle in radians, loaded from compass heading
	double theta_c = vehicle_data->position_heading;

	//load waypoint position
	//need to make this be loaded from our maps csv file
       // double x2 = 262.109970163;	//In front of seargent arts
	//double y2 = 640.4382995025;
	
	double x2 = 239.3933226736;	//walkway by FMA towards annex breezway
	double y2 = 622.4247213858;

	//double x2 = 272.5103629905;	//Below walkway
	//double y2 = 587.5095144198;

	//calculate waypoint angle
	double theta_w = waypoint_math.calculateThetaW(x1,y1,x2,y2);

	//calculate steering angle
	double theta_s = waypoint_math.calcSteerAngle(theta_c, theta_w, k);	
	//std::cout<<" theta_s"<<theta_s<<std::endl;
	
	//calculate distance to waypoint
	double distance = waypoint_math.computeDistance(x1,y1,x2,y2);

	//send out debug messages
	ROS_DEBUG_STREAM("theta_s" ); 
	ROS_INFO_STREAM("theta_s " << theta_s);
	ROS_INFO_STREAM("Vehicle Point (x,y)" << x1 << "," << y1);
	ROS_INFO_STREAM("Waypoint (x,y)" << x2 << ","<< y2);
	ROS_INFO_STREAM("Distance (meters) " << distance);
	//ROS_INFO_STREAM("Latitude Constant " << lat_const << ", " << long_const);
	//send results to vehicle_data
	vehicle_data->speed_cmd=1;
	vehicle_data->brake_cmd=0;
	vehicle_data->steer_cmd=theta_s;
	//std::cout<<"steer_cmd"<<vehicle_data->steer_cmd<<std::endl;

	//return

}



bool DrivePathState::debugState(StateMachine* state_machine) {

    std::vector<std::string> line_desc;
    std::vector<VehicleEvents> event_lines;

    line_desc.push_back("PATH FINISHED");
    event_lines.push_back(PATH_FINISHED);

    return queryUserForTransition("DRIVE_PATH", state_machine, line_desc,  event_lines);
}

#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <state_machine/waypoint_math.h>

// Calculates the angle to the waypoint relative to east.
// temptheta_w is the quadrant relative angle to the waypoint
// x1, y1, are the local position of the car// TODO: Correct to local coordinate system.
// x2, y2, are the local position of the waypoint.
//  return theta_w is the angle to the waypoint. Measured CCW from East.
double WaypointMath::calculateThetaW(double x1, double y1, double x2,
        double y2) {
    double theta_w = atan((y2 - y1) / (x2 - x1));
    
    if (x2 > x1 && y2 > y1) {
        return theta_w;
    } // quadrant I
    else if (x2 < x1 && y2 > y1) {
        return (theta_w + pi_num);
    } // quadrant II
    else if (x2 < x1 && y2 < y1) {
        return (theta_w + pi_num);
    } // quadrant III
    else {
        return (theta_w + (2 * pi_num));
    } // quadrant IV (x2 > x1 && y2 < y1)
    
} //end calculateThetaW

// Determines which direction the vehicle should turn in order to reach its destination
// theta_c is the car's heading relative to E, measured CCW.
// theta_w is the waypoint heading relative to E, measured CCW.
// theta_d is the angle from the car heading to the waypoint, measured CCW. We are pointed at the waypoint when theta_d = 0.
// k is the proportional gain for steering commands
// returns a steering angle (in radians). A positive steering command is CW and turns the vehicle right. 
double WaypointMath::calcSteerAngle(double &theta_c, double &theta_w,
        double k) {
    //calculate the difference in car heading and waypoint angle
    double theta_d = (theta_c * (pi_num / 180) - theta_w);
    
    
    int negVar = 1; //variable that controls left and right turns. If negVar=1, we turn right. If negVar=-1, we turn left.
    
    if (theta_d > 0) {
        if (theta_d < pi_num) {
            ROS_DEBUG_STREAM( "Turn Right"  );
        } else {
            negVar = -1; ROS_DEBUG_STREAM( "Turn Left" );
        }
    } else {
        if (std::abs(theta_d) < pi_num) {
            negVar = -1; ROS_DEBUG_STREAM( "Turn Left" );
        } else {
            ROS_DEBUG_STREAM( "Turn Right" );
        }
    }
    
    if (std::abs(theta_d) > max_steer) {
        return (negVar * max_steer);
        //return(1);
    } else {
        return (negVar * k * std::abs(theta_d));
        //return(2);
    }
} // end calcSteerAngle


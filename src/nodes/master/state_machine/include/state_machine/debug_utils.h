#ifndef DEBUG_UTILS_H 
#define DEBUG_UTILS_H

//file containes functions for debugging states machine code

#include <iostream>
#include <vector>
#include <string>
#include <assert.h>

#include <ros/ros.h>

#include "state_machine/state_machine.h"

//testing function. takes a list of possible events and lets the user choose one 
//  of them for a state transsition. 
//Returns true if the program is to continue as normal, and false if we are to 
//  go directly to the next state 
inline bool queryUserForTransition(std::string current_state,
        StateMachine* state_machine, std::vector<std::string>& line_desc,
        std::vector<VehicleEvents>& event_lines) {
    
    //declare we are in debug mode
    ROS_DEBUG_STREAM( "Debug Mode Active (query user for transition): ");
    
    //make sure input is valid
    if (line_desc.size() != event_lines.size()) {
        ROS_DEBUG_STREAM(
                 "Error invalid data given to testing function. Vector sizes mismatched."
                );
        assert(false);
    }
    
    ROS_DEBUG_STREAM("Current state is: " << current_state );
    
    //input loop. keep asking until we get a valid answer. 
    std::string answer_str;
    int answer_int;
    std::string::size_type size;
    
    bool valid_answer = false;
    do {
        ROS_DEBUG_STREAM( "Please select one of the options below:" );
        
        //print options
        unsigned int i;
        for (i = 0; i < line_desc.size(); i++) {
            ROS_DEBUG_STREAM( i << ": " << "Send state transition event - "
                    << line_desc.at(i) );
        }
        
        //print normal operation's option
        ROS_DEBUG_STREAM( i << ": continue normal program operations." );
        
        //query for input
        ROS_DEBUG_STREAM( "Please enter the number of the option above: ");
        std::cin >> answer_str;
        
        answer_int = std::stoi(answer_str, &size);
        
        //no number was given or answer is not one of the options 
        if (size == 0 || answer_int < 0 || (unsigned int) answer_int > i) {
            continue;
        }
        
        //answer is the last operation let the loop end normally
        if ((unsigned int) answer_int == i) {
            valid_answer = true;
            
            //otherwise the answer is one of the events so we trigger that event
        } else {
            state_machine->internalEvent(event_lines.at(answer_int));
            valid_answer = true;
            return false; //tell calling function to end immediately
        }
        
    } while (!valid_answer);
    
    return true; //tell calling function to continue as normal.
}

#endif

#ifndef DEBUG_UTILS_H 
#define DEBUG_UTILS_H

//file containes functions for debugging states machine code

#include <iostream>
#include <vector>
#include <string>
#include <assert.h>

#include "state_machine/state_machine.h"


//testing function. takes a list of possible events and lets the user choose one 
//  of them for a state transsition. 
//Returns true if the program is to continue as normal, and false if we are to 
//  go directly to the next state 
inline bool queryUserForTransition(std::string current_state, StateMachine* state_machine, 
  std::vector<std::string>& line_desc, std::vector<VehicleEvents>& event_lines){

    //declare we are in debug mode
    std::cout << "Debug Mode Active (query user for transition): " << std::endl;

    //make sure input is valid
    if (line_desc.size() != event_lines.size()){
        std::cout << "Error invalid data given to testing function. Vector sizes mismatched." << std::endl;
        assert(false);
    }

    std::cout << "Current state is: " << current_state << std::endl;   

    //input loop. keep asking until we get a valid answer. 
    std::string answer_str;
    int answer_int;
    std::string::size_type size;
    
    bool valid_answer = false;
    do {
        std::cout << "Please select one of the options below:" << std::endl;
    
        //print options
        unsigned int i;
        for (i = 0; i < line_desc.size(); i++) {
            std::cout << i << ": " << "Send state transition event - " 
                      << line_desc.at(i) << std::endl; 
        }

        //print normal operation's option
        std::cout << i << ": continue normal program operations." << std::endl;
   
        //query for input
        std::cout << "Please enter the number of the option above: ";
        std::cin >> answer_str;

        answer_int = std::stoi (answer_str,&size);

        //no number was given or answer is not one of the options 
        if (size == 0 || answer_int < 0 || (unsigned int) answer_int > i) {
            continue;
        }

        //answer is the last operation let the loop end normally
        if ((unsigned int)answer_int == i) {
            valid_answer = true;

        //otherwise the answer is one of the events so we trigger that event
        } else {
            state_machine->internalEvent(event_lines.at(answer_int));
            valid_answer = true;
            return false; //tell calling function to end imediatly
        }

    } while(!valid_answer);
    
    return true; //tell calling function to continue as normal.
}


#endif

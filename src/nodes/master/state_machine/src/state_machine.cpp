
#include "state_machine/state_machine.h"
#include "state_machine/state_interfaces.h"

#include <iostream>

StateMachine::StateMachine() {

    //TODO: for some reason if I don't expand the two vectors here they will not
    //    be properly populated
    StateTableRow empty_row(INVALID_STATE, INVALID_STATE);
    state_transition_table.push_back(empty_row);

    state_objects.push_back(nullptr);

    //setup table transition table. If adding new states add another row here
    //                 Event         : Current State   : Next State
    buildTableRow(INITIALIZE_FINISHED, INITIALIZE      , DETECT_STATION         );
    buildTableRow(STATION_DETECTED   , DETECT_STATION  , WAIT_FOR_COMMAND       );
    buildTableRow(SHUTDOWN_REQUESTED , WAIT_FOR_COMMAND, SHUTDOWN               );
    buildTableRow(STATION_REQUESTED  , WAIT_FOR_COMMAND, INITIAL_OBSTACLE_DETECT);
    //                    Event               : Current State          : Next State
    buildTableRow(NO_INITIAL_OBSTICAL_DETECTED, INITIAL_OBSTACLE_DETECT, RELEASE_BRAKE   );
    buildTableRow(INITIAL_OBSTICAL_DETECTED   , INITIAL_OBSTACLE_DETECT, WAIT_FOR_COMMAND);
    //                 Event        : Current State    : Next State
    buildTableRow(BREAK_RELEASED    , RELEASE_BRAKE    , CHECK_DESTINATION   );
    buildTableRow(BREAK_LOCKED      , PARK             , WAIT_FOR_COMMAND    );
    buildTableRow(NOT_AT_DESTINATION, CHECK_DESTINATION, CHECK_STOP_SIGN     );
    buildTableRow(AT_DESTINATION    , CHECK_DESTINATION, PARK                );
    //                 Event        : Current State    : Next State
    buildTableRow(NO_STOP_SIGN      , CHECK_STOP_SIGN     , DRIVE_PATH          );
    buildTableRow(STOP_SIGN         , CHECK_STOP_SIGN     , PERFORMING_TAXI_STOP);
    buildTableRow(TAXI_STOP_FINISHED, PERFORMING_TAXI_STOP, DRIVE_PATH          );
    buildTableRow(PATH_FINISHED     , DRIVE_PATH          , CHECK_DESTINATION   );


    //create state_objects
    addState(INVALID_STATE          , nullptr);
    addState(INITIALIZE             , new InitilizeState);
    addState(DETECT_STATION         , new DetectStationState);
    addState(WAIT_FOR_COMMAND       , new WaitForCommandState);
    addState(SHUTDOWN               , new ShutdownState);
    addState(INITIAL_OBSTACLE_DETECT, new InitialObstacleDetectState);
    addState(RELEASE_BRAKE          , new ReleaseBrakeState);
    addState(PARK                   , new ParkState);
    addState(CHECK_DESTINATION      , new CheckDestinationState);
    addState(CHECK_STOP_SIGN        , new CheckStopSignState);
    addState(PERFORMING_TAXI_STOP   , new PerformingTaxiStopState);
    addState(DRIVE_PATH             , new DrivePathState);

    //lock table so we cannot add to it anymore
    lock_transition_table = true;

    //set current state to the first valid state
    current_state = INITIALIZE;
}


StateMachine::~StateMachine() {
    for (unsigned int i = 0; i < state_objects.size()-1; i++) {
        delete state_objects[i];
    }
}


//adds a row to the transition table. takes the event, starting state for the 
//    event, and the next state to which we will transition upon the event
void StateMachine::buildTableRow(VehicleEvents event, VehicleStates valid_state,
                                            VehicleStates next_state) {

    //block adding new tables is the table is locked
    if (lock_transition_table) {
        //TODO: ouput debug message
        return;
    }

    //make sure the given event has a place in the transition table
    while(state_transition_table.size()-1 < event) {

        StateTableRow empty_row(INVALID_STATE, INVALID_STATE);
        state_transition_table.push_back(empty_row);

    }

    //setup event table row and add it to the table
    StateTableRow new_row(valid_state, next_state);

    state_transition_table[event] = new_row;

}


//adds a state to the correct location
void StateMachine::addState(VehicleStates state, AbstractState* state_obj){

    //block adding new tables is the table is locked
    if (lock_transition_table) {
        //TODO: ouput debug/warning message
        return;
    }

    //make sure the given event has a place in the transition table
    while(state_objects.size()-1 < state) {

        state_objects.push_back(nullptr);

    }

    //place state at correct location 
    state_objects[state] = state_obj;

}


//called to trigger a transition by a state
void StateMachine::internalEvent(VehicleEvents event){

    VehicleStates source = state_transition_table[event].source_state;
    VehicleStates next = state_transition_table[event].next_state;

    if (current_state == source) {
        //change current state to the new state
        current_state = next;

        //TODO: update GUI debug screen with new state info
    } else {
        //send out debug/warning message about an invalid event call
    }
}



//called to trigger a transition by something other then a state.
//TODO: Not sure we even need this function
void StateMachine::externalEvent(VehicleEvents event){
    internalEvent(event);
}



//a tick has passed, the state machine updates the current state
void StateMachine::tick(VehicleData * vehicle_data){

    state_objects[current_state]->tick(this, vehicle_data);
}
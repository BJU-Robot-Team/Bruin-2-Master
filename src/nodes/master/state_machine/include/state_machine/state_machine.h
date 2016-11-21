#ifndef STATE_MACHINE_H 
#define STATE_MACHINE_H

#include <vector>

#include "state_machine/vehicle_data.h"


//If you change the states or events you need to change the state machine 
//   constructor which builds the state table transition

//list of possible machine states
enum VehicleStates {
    INVALID_STATE,
    INITIALIZE,
    SHUTDOWN,
    DETECT_STATION,
    WAIT_FOR_COMMAND,
    INITIAL_OBSTACLE_DETECT,
    RELEASE_BRAKE,
    PARK,
    CHECK_DESTINATION,
    CHECK_STOP_SIGN,
    PERFORMING_TAXI_STOP,
    DRIVE_PATH,
    WAITING_FOR_OBSTACLE
};

//events that can cause a state transition
enum VehicleEvents {
    INITIALIZE_FINISHED,
    STATION_DETECTED,
    SHUTDOWN_REQUESTED,
    STATION_REQUESTED,
    NO_INITIAL_OBSTICAL_DETECTED,
    INITIAL_OBSTICAL_DETECTED,
    BREAK_RELEASED,
    BREAK_LOCKED,
    NOT_AT_DESTINATION,
    AT_DESTINATION,
    NO_STOP_SIGN,
    STOP_SIGN,
    TAXI_STOP_FINISHED,
    PATH_FINISHED
    
};

struct StateTableRow {
    VehicleStates source_state; //state we are in when the event occures
    VehicleStates next_state; //state we will transition to after the event

    StateTableRow(VehicleStates source, VehicleStates next) 
      : source_state(source), next_state(next)
    {};
};

class AbstractState;

//StateMachine's primary purpose is to insure that events can only trigger a 
//  transition for the correct state. 
class StateMachine {
    //allows AbstractState and it's children to access "InternalEvent"
    //friend AbstractState;

  private:

    //table describing what states an event can trigger 
    std::vector< StateTableRow > state_transition_table {};
    bool lock_transition_table = false; //table can only be added to at the beginning 

    //the current state
    VehicleStates current_state;

    //list of state objects
    std::vector< AbstractState * > state_objects;

    //adds a row to the state table
    void buildTableRow(VehicleEvents event, VehicleStates valid_state,
                                            VehicleStates next_state);

    //adds a state to the correct location
    void addState(VehicleStates state, AbstractState* state_obj);

  public:
    //weather the program is in debug mode. if so the states will allow user input for testing purposes
    bool debug_mode = false;

    StateMachine();
    ~StateMachine();

    //called to trigger a transition by a state
    void internalEvent(VehicleEvents event);

    //called to trigger a transition by something other then a state.
    void externalEvent(VehicleEvents event);

    //a tick has passed, the state machine updates the current state
    void tick(VehicleData * vehicle_data);
    
};

#endif

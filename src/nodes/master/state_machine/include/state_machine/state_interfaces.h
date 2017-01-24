#ifndef STATE_INTERFACES_H 
#define STATE_INTERFACES_H

#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"

//each state is basically just a wrapper for a function run on each tick
class AbstractState {
  public:
    AbstractState() {};
    virtual ~AbstractState() {};
    
    virtual void tick(StateMachine* state_machine, VehicleData* vehicle_data) = 0;
};



//All state interfaces are defined here

//Initialize 
class InitilizeState : public AbstractState {
  public:
    ~InitilizeState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class ShutdownState : public AbstractState {
  public:
    ~ShutdownState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class DetectStationState : public AbstractState {
  public:
    ~DetectStationState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class WaitForCommandState : public AbstractState {
  public:
    ~WaitForCommandState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class InitialObstacleDetectState : public AbstractState {
  public:
    ~InitialObstacleDetectState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class ReleaseBrakeState : public AbstractState {
  public:
    ~ReleaseBrakeState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class ParkState : public AbstractState {
  public:
    ~ParkState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class CheckDestinationState : public AbstractState {
  public:
    ~CheckDestinationState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class CheckStopSignState : public AbstractState {
  public:
    ~CheckStopSignState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class PerformingTaxiStopState : public AbstractState {
  public:
    ~PerformingTaxiStopState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class DrivePathState : public AbstractState {
  public:
    ~DrivePathState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class FollowState : public AbstractState {
  public:
    ~FollowState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};
class JoystickState : public AbstractState {
  public:
    ~JoystickState() {};
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};


#endif

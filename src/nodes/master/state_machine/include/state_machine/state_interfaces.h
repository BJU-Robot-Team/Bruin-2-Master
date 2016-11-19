#ifndef STATE_INTERFACES_H 
#define STATE_INTERFACES_H

#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"

//each state is basically just a wrapper for a function run on each tick
class AbstractState {
  public:
    AbstractState() {};

    virtual void tick(StateMachine* state_machine, VehicleData* vehicle_data) = 0;
};



//All state interfaces are defined here

//Initialize 
class InitilizeState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class ShutdownState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class DetectStationState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class WaitForCommandState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class InitialObstacleDetectState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class ReleaseBrakeState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class ParkState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class CheckDestinationState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class CheckStopSignState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class PerformingTaxiStopState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class DrivePathState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

class WaitingForObstacleState : AbstractState {
  public:
    void tick(StateMachine* state_machine, VehicleData* vehicle_data);
    bool debugState(StateMachine* state_machine);
};

#endif

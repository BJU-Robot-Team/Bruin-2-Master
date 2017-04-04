#ifndef VEHICLE_DATA_H 
#define VEHICLE_DATA_H

#include <string>
#include <vector>
#include <iostream>

#include "state_machine/waypoint_map.h"


//state for a single relay board device - obsolete, use relay_mask instead?
// mask is better because we can switch multiple relays simultaneously
// it is more complicated because we have to maintain the state of them all to do this
struct RelayDeviceStateData {
    std::string device_type;
    int device_number;
    std::string device_state;

    RelayDeviceStateData(std::string device_type, int device_number) 
      : device_type(device_type), device_number(device_number) {
        device_state = "unknown";
    }
};

struct VehicleData {

    //objects that keep track of relay information
    std::vector<RelayDeviceStateData> relay_states;
    std::vector<RelayDeviceStateData> gpio_states;

    //Waypoint map object storing all possible paths
    WaypointMap* waypoint_map;

    //gps data for current location
    double position_latitude  = 0;
    double position_longitude = 0;

    //gps origin, needs to be loaded from ini file
    double origin_lat = 34.8686574
    double origin_long=-82.36457

    //Vehicle heading
    double position_heading = 0;

    //Folow target data
    double follow_direction = 0;
    double follow_distance = 0;
    bool follow_valid = false;

    // Actuator commands
    double steer_cmd = 0;
    double brake_cmd = 0;
    double speed_cmd = 0;
    uint16_t relay_mask = 0;

    bool shutdown = false;
    char char_input = 0;	

    VehicleData() {

        const int number_of_relays = 16;
        for (int i = 0; i < number_of_relays; i++) {
            relay_states.push_back(RelayDeviceStateData("relay", i));
        }

        const int number_of_gpios = 6;
        for (int i = 0; i < number_of_gpios; i++) {
            gpio_states.push_back(RelayDeviceStateData("gpio", i));
        }

    }

};

#endif

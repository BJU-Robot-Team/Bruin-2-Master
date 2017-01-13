#ifndef VEHICLE_DATA_H 
#define VEHICLE_DATA_H

#include <string>
#include <vector>
#include <iostream>


//state for a single relay board device
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

    std::vector<RelayDeviceStateData> relay_states;
    std::vector<RelayDeviceStateData> gpio_states;

    //gps data for current location
    double position_latitude  = 0;
    double position_longitude = 0;

    //Vehicle heading
    double position_heading = 0;

    //Folow target data
    double follow_direction = 0;
    double follow_distance = 0;
    bool follow_valid = false;

    // Actuator commands
    double steer_cmd;
    double brake_cmd;
    double speed_cmd;	

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

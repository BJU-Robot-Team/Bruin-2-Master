

#include "relay_board/relay_board_commands.h"
#include "relay_board/ros_interface.h"

#include <serial/serial.h>


int main(int argc, char **argv) {

    //function defined in ros_interface header
    startROS(argc, argv);

    ROSInterface ros_interface;

    std::string port = "/dev/ttyAMC0";
    unsigned long baud = 9600;

    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
    RelayBoardCommands relay_command_interface(my_serial);

    //setup main loop
    std::string device_type = "";
    int device_num = -1;
    std::string command = "";
    std::string state = "";

    while (ros_interface.isNodeRunning()) {

        //look for a message to react to. pass values by reference
        if (ros_interface.pollMessages(device_type, device_num, command)) {

            //device is a relay
            if (device_type == "relay") {

                //TODO: we should probebly read the state to make sure it actually changed
                //   since a normal command to the relay board returns nothing

                if (command == "on") {
                    relay_command_interface.relayOn(my_serial, device_num);
                    state = "on";

                } else if (command == "off") {
                    relay_command_interface.relayOff(my_serial, device_num);
                    state = "off"; 
                } else if (command == "read") {
                    state = relay_command_interface.relayRead(my_serial, device_num);
                    state = state.substr(state.find(" ") + 1); 
                }


            //device is a gpio command
            } else {

                 if (command == "read") {
                    state = relay_command_interface.gpioRead(my_serial, device_num);
                    state = state.substr(state.find(" ") + 1); 

                }

            }

            //send message to anyone listening so they know the sate has changed
            ros_interface.publishMessages(device_type, device_num, state);
 
            //reset values
            device_type = "";
            device_num = -1;
            command = "";
            state = "";

        }

    }

}

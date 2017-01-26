#ifndef RELAY_BOARD_COMMANDS_H 
#define RELAY_BOARD_COMMANDS_H


#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include <string>

#include <iostream>

class RelayBoardCommands {

  private:

    serial::Serial* serial_test;

  public:

    bool debug_mode = false;


    RelayBoardCommands(serial::Serial *my_serial);

    //print out debuginfo for a serial transaction
    void debugPrint(std::string command_str, size_t bytes_wrote, std::string result);

    //write the given command to serial then read the next message
    std::string serialTransaction(serial::Serial *my_serial, std::string command_str);

    //gets the relay board software version
    std::string version(serial::Serial *my_serial);

    //turn on a specified relay on the board
    std::string relayOn(serial::Serial *my_serial, unsigned int relay_id);

    //turn on a specified relay on the board
    std::string writeAll(serial::Serial *my_serial, unsigned int mask);

    //turn off a specified relay on the board
    std::string relayOff(serial::Serial *my_serial, unsigned int relay_id);

    //get the current status of a specified relay on the board
    std::string relayRead(serial::Serial *my_serial, unsigned int relay_id);

    //get the current status of a specified relay on the board
    std::string gpioRead(serial::Serial *my_serial, unsigned int gpio_id);

};


#endif

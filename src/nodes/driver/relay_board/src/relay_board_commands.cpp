
#include "relay_board/relay_board_commands.h"

#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include <string>


//setup constructor
RelayBoardCommands::RelayBoardCommands() { //std::string port){
    //set variables
    //unsigned long baud = 9600;

    // port, baudrate, timeout in milliseconds
    //my_serial = serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
}


//define valid commands

//gets the relay board software version
std::string RelayBoardCommands::version(serial::Serial& my_serial) {
    //send command
    std::string command_str = "ver";
    size_t bytes_wrote = my_serial.write(command_str);

    //read response
    std::string result = my_serial.read(command_str.length()+1);

    return result;
}


//turn on a specified relay on the board
std::string RelayBoardCommands::relayOn(serial::Serial& my_serial, unsigned int relay_id) {
    //send command
    std::string command_str = "relay on "+relay_id;
    size_t bytes_wrote = my_serial.write(command_str);

    //read response
    std::string result = my_serial.read(command_str.length()+1);

    return result;
}


//turn off a specified relay on the board
std::string RelayBoardCommands::relayOff(serial::Serial& my_serial, unsigned int relay_id) {
    //send command
    std::string command_str = "relay off "+relay_id;
    size_t bytes_wrote = my_serial.write(command_str);

    //read response
    std::string result = my_serial.read(command_str.length()+1);

    return result;
}

//get the current status of a specified relay on the board
std::string RelayBoardCommands::relayRead(serial::Serial& my_serial, unsigned int relay_id) {
    //send command
    std::string command_str = "relay read "+relay_id;
    size_t bytes_wrote = my_serial.write(command_str);

    //read response
    std::string result = my_serial.read(command_str.length()+1);

    return result;
}


//get the current status of a specified relay on the board
std::string RelayBoardCommands::gpioRead(serial::Serial& my_serial, unsigned int gpio_id) {
    //send command
    std::string command_str = "gpio read "+gpio_id;
    size_t bytes_wrote = my_serial.write(command_str);

    //read response
    std::string result = my_serial.read(command_str.length()+1);

    return result;
}


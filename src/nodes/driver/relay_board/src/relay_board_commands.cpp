
#include "relay_board/relay_board_commands.h"

#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include <iostream>
#include <string>


//setup constructor
RelayBoardCommands::RelayBoardCommands() { //std::string port){

    //TODO: for some reason we cannot use operator= with the Serial class so we 
    //  have to pass it to each function instead of having it in a member variable
    //  Fix/"get around" this issue and remove passing Serial class by referance
    
    //set variables
    //unsigned long baud = 9600;

    // port, baudrate, timeout in milliseconds
    //my_serial = serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
}


//some helper functions

//print out debuginfo for a serial transaction
void RelayBoardCommands::debugPrint(std::string command_str, size_t bytes_wrote, std::string result) {

    if (debug_mode) {
        //report
        std::cout << "Bytes written: " << bytes_wrote;
        //std::cout << ", Command written: " << command_str; //TODO: only send the string that was actually written
        std::cout << ", Bytes read: " << result.length();
        std::cout << ", String read: " << result << std::endl;
    }
}


//write the given command to serial then read the next message
std::string RelayBoardCommands::serialTransaction(serial::Serial& my_serial, std::string command_str) {

    size_t bytes_wrote = my_serial.write(command_str);

    //read response
    std::string result = my_serial.read(command_str.length()+1);

    debugPrint(command_str, bytes_wrote, result);

    return result;
}


//define valid commands

//gets the relay board software version
std::string RelayBoardCommands::version(serial::Serial& my_serial) {
    //send command
    std::string command_str = "ver";

    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);

    return result;
}


//turn on a specified relay on the board
std::string RelayBoardCommands::relayOn(serial::Serial& my_serial, unsigned int relay_id) {
    //send command
    std::string command_str = "relay on "+relay_id;
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);

    return result;
}


//turn off a specified relay on the board
std::string RelayBoardCommands::relayOff(serial::Serial& my_serial, unsigned int relay_id) {
    //send command
    std::string command_str = "relay off "+relay_id;
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);

    return result;
}

//get the current status of a specified relay on the board
std::string RelayBoardCommands::relayRead(serial::Serial& my_serial, unsigned int relay_id) {
    //send command
    std::string command_str = "relay read "+relay_id;
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);

    return result;
}


//get the current status of a specified relay on the board
std::string RelayBoardCommands::gpioRead(serial::Serial& my_serial, unsigned int gpio_id) {
    //send command
    std::string command_str = "gpio read "+gpio_id;
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);

    return result;
}


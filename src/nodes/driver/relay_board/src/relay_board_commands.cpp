// Relay Board Commands
// V1.0
#include "relay_board/relay_board_commands.h"

#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <unistd.h>
#include "ros/ros.h"

//setup constructor
RelayBoardCommands::RelayBoardCommands(serial::Serial *my_serial) { //std::string port){
    //TODO: for some reason we cannot use operator= with the Serial class so we 
    //  have to pass it to each function instead of having it in a member variable
    //  Fix/"get around" this issue and remove passing Serial class by referance
    
    //set variables
    //unsigned long baud = 9600;
    
    // port, baudrate, timeout in milliseconds
    //my_serial = serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
    
    serial_test = my_serial;

    //send an initial character to start the prompt on the relay device
    std::string result = serialTransaction(my_serial, "\r");
}

//some helper functions

//print out debuginfo for a serial transaction
void RelayBoardCommands::debugPrint(std::string command_str, size_t bytes_wrote, std::string result) {
    
    if (debug_mode) {
        //report
         ROS_DEBUG_STREAM( "Bytes written: " << bytes_wrote);

         ROS_DEBUG_STREAM( ", Bytes read: " << result.length());
         ROS_DEBUG_STREAM( ", String read: " << result );
    }
}

//write the given command to serial then read the next message
std::string RelayBoardCommands::serialTransaction(serial::Serial *my_serial, std::string command_str) {
    //add needed characters to the end
    command_str = command_str + "\n\r";
    
    size_t bytes_written = my_serial->write(command_str);
    
    //usleep(100);
    
    //read response
    std::string result = my_serial->read(30);
    size_t found = result.find('\n');
    result = result.substr(found + 1);
    found = result.find('\n');
    result = result.substr(found + 1);
    found = result.find('\n');
    result = result.substr(1, found);
    
    debugPrint(command_str, bytes_written, result);
    
    return result;
}

//define valid commands

//gets the relay board software version
std::string RelayBoardCommands::version(serial::Serial *my_serial) {
    //send command
    std::string command_str = "ver";
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);
    
    return result;
}

//turn on a specified relay on the board
std::string RelayBoardCommands::relayOn(serial::Serial *my_serial, unsigned int relay_id) {
    //send command
    std::string command_str = "relay on " + std::to_string(relay_id);
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);
    
    return result;
}

//turn on/off all relays
std::string RelayBoardCommands::writeAll(serial::Serial *my_serial, unsigned int mask) {
    std::stringstream sstream;
    sstream << std::hex << std::setw(4) << std::setfill('0') << mask;
    std::string wresult = sstream.str();
    //send command
    std::string command_str = "relay writeall " + wresult + "\r";
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);
    
    return result;
}

//turn off a specified relay on the board
std::string RelayBoardCommands::relayOff(serial::Serial *my_serial, unsigned int relay_id) {
    //send command
    std::string command_str = "relay off " + std::to_string(relay_id);
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);
    
    return result;
}

//get the current status of a specified relay on the board
std::string RelayBoardCommands::relayRead(serial::Serial *my_serial, unsigned int relay_id) {
    //send command
    std::string command_str = "relay read " + std::to_string(relay_id);
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);
    
    return result;
}

//get the current status of a specified relay on the board
std::string RelayBoardCommands::gpioRead(serial::Serial *my_serial, unsigned int gpio_id) {
    //send command
    std::string command_str = "gpio read " + std::to_string(gpio_id);
    
    //send command and recive the response
    std::string result = serialTransaction(my_serial, command_str);
    
    return result;
}

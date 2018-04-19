#include <iostream>
#include <string>
#include "ros/ros.h"
#include "relay_board/relay_board_commands.h"

#include <serial/serial.h>

int main(void) {
    
    std::string port = "/dev/ttyACM0";
    unsigned long baud = 9600;
    
    serial::Serial *my_serial;
    my_serial = new serial::Serial(port, baud,
            serial::Timeout::simpleTimeout(1000));
    RelayBoardCommands relay_command_interface(my_serial);
    my_serial->setFlowcontrol(serial::flowcontrol_none);
    
    std::string answer_str;
    int answer_int;
    std::string::size_type size;
    int relay_or_gpio = 0;
    
    do {
        ROS_DEBUG_STREAM( "Please choose an option below:" );
        ROS_DEBUG_STREAM( "0: request version" );
        ROS_DEBUG_STREAM( "1: read relay" );
        ROS_DEBUG_STREAM( "2: turn relay on" );
        ROS_DEBUG_STREAM( "3: turn relay off" );
        ROS_DEBUG_STREAM( "4: read gpio" );
        
        std::string answer_str;
        std::cin >> answer_str;
        
        answer_int = std::stoi(answer_str, &size);
        
        //no number was given or answer is not one of the options 
        if (size == 0 || answer_int < 0 || (unsigned int) answer_int > 4) {
            ROS_ERROR_STREAM(  "####> Error: Invalid input <####" );
            continue;
        }
        
        //get secondary input
        if (answer_int >= 1 && answer_int <= 3) {
            ROS_DEBUG_STREAM( "Enter which relay to effect: " );
            std::cin >> relay_or_gpio;
        } else if (answer_int == 4) {
            ROS_DEBUG_STREAM( "Enter which GPIO to effect: " );
            std::cin >> relay_or_gpio;
        } else { //we are doing a command that does not have input
            relay_or_gpio = -1;
        }
        
        switch (answer_int) {
            case 0: // version command
                relay_command_interface.version(my_serial);
                break;
                
            case 1: // read relay
                relay_command_interface.relayRead(my_serial,
                        (unsigned int) relay_or_gpio);
                break;
                
            case 2: // turn relay on
                relay_command_interface.relayOn(my_serial,
                        (unsigned int) relay_or_gpio);
                break;
                
            case 3: // turn relay off
                relay_command_interface.relayOff(my_serial,
                        (unsigned int) relay_or_gpio);
                break;
                
            case 4: // read gpio
                relay_command_interface.gpioRead(my_serial,
                        (unsigned int) relay_or_gpio);
                break;
                
            default:
                 ROS_DEBUG_STREAM( "Invalid command." );
        }
        
    } while (true);
}


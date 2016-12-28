#include <iostream>
#include <string>

#include "relay_board/relay_board_commands.h"

#include <serial/serial.h>


int main(void) {

    std::string port = "/dev/ttyACM0";
    unsigned long baud = 9600;

    serial::Serial *my_serial;
    my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));
    RelayBoardCommands relay_command_interface(my_serial);
    my_serial->setFlowcontrol(serial::flowcontrol_none);

    std::string answer_str;
    int answer_int;
    std::string::size_type size;
    int relay_or_gpio = 0; 

    do {
        std::cout << "Please choose an option below:" << std::endl;
        std::cout << "0: request version" << std::endl;
        std::cout << "1: read relay" << std::endl;
        std::cout << "2: turn relay on" << std::endl;
        std::cout << "3: turn relay off" << std::endl;
        std::cout << "4: read gpio" << std::endl;

        std::string answer_str;
        std::cin >> answer_str;

        answer_int = std::stoi(answer_str, &size);

        //no number was given or answer is not one of the options 
        if (size == 0 || answer_int < 0 || (unsigned int) answer_int > 4) {
            std::cout << "####> Error: Invalid input <####" << std::endl;
            continue;
        }

        //get secondary input
        if ( answer_int >= 1 && answer_int <= 3 ) {
            std::cout << "Enter which relay to effect: " << std::endl;
            std::cin >> relay_or_gpio;
        } else if (answer_int == 4) {
            std::cout << "Enter which GPIO to effect: " << std::endl;
            std::cin >> relay_or_gpio;
        } else { //we are doing a command that does not have input
            relay_or_gpio = -1;
        }
        
        switch(answer_int) {
            case 0: // version command
                relay_command_interface.version(my_serial);
                break;

            case 1: // read relay
                relay_command_interface.relayRead(my_serial, (unsigned int) relay_or_gpio);
                break;

            case 2: // turn relay on
                relay_command_interface.relayOn(my_serial, (unsigned int) relay_or_gpio);
                break;

            case 3: // turn relay off
                relay_command_interface.relayOff(my_serial, (unsigned int) relay_or_gpio);
                break;

            case 4: // read gpio
                relay_command_interface.gpioRead(my_serial, (unsigned int) relay_or_gpio);
                break;

            default:
                std::cout << "Invalid command." << std::endl;
        }
    
    } while(true);
}



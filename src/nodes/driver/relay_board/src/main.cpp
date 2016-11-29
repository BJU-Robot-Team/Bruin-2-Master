

#include "relay_board/relay_board_commands.h"

#include <serial/serial.h>


int main(void) {

    std::string port = "";
    unsigned long baud = 9600;

    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
    RelayBoardCommands relay_command_interface;
}

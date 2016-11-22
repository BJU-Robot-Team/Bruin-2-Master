#include <iostream>

#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include <string>

#include <iostream>


int main(void) {

    //set variables
    std::string port = "";
    unsigned long baud = 9600;

    // port, baudrate, timeout in milliseconds
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));

    //send command
    std::string command_str = "ver";
    size_t bytes_wrote = my_serial.write(command_str);

    //read response
    std::string result = my_serial.read(command_str.length()+1);

    //report
    std::cout << "Bytes written: ";
    std::cout << bytes_wrote << ", Bytes read: ";
    std::cout << result.length() << ", String read: " << result << std::endl;

    return 0;
}



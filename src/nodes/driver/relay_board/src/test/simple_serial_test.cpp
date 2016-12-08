#include <iostream>

#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include <string>

#include <iostream>

#include <unistd.h>


int main(void) {

    //set variables
    std::string port = "/dev/ttyACM0";
    unsigned long baud = 9600;

    // port, baudrate, timeout in milliseconds
    serial::Serial my_serial(port, baud, serial::Timeout::simpleTimeout(1000));
    my_serial.setStopbits(serial::stopbits_one);
    my_serial.setParity(serial::parity_none);
    my_serial.setFlowcontrol(serial::flowcontrol_none);


    //send initial carachter to get prompt
    std::string command_str = "\r";
    size_t bytes_wrote = my_serial.write(command_str);
    my_serial.read(command_str.length()+1);

    //send command
    command_str = "ver";
    bytes_wrote = my_serial.write(command_str+"\r\n");

    usleep(10000);

    //read response
    std::string result = my_serial.read(command_str.length()+1);

    //report
    std::cout << "Bytes written: ";
    std::cout << bytes_wrote << ", Bytes read: ";
    std::cout << result.length() << ", String read: " << result << std::endl;

    return 0;
}



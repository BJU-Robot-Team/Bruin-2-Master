#include <iostream>

#include <serial/serial.h>
#include <serial/utils/serial_listener.h>

#include "ros/ros.h"
using namespace serial;
using namespace serial::utils;

void default_handler(std::string token) {
  ROS_DEBUG_STREAM( "default_handler got a: " << token );
}

void callback(std::string token) {
  ROS_DEBUG_STREAM( "callback got a: " << token );
}

int run() {
  // Assuming this device prints the string 'pre-substr-post\r' at 100Hz
  Serial serial("/dev/tty.usbserial-A900cfJA", 115200);

  SerialListener listener;
  listener.startListening(serial);

  // Set the tokenizer
  //  This is the same as the default delimeter, so an explicit call to
  //  setTokenizer is not necessary if your data is \r delimited.
  //  You can create your own Tokenizer as well.
  listener.setTokenizer(SerialListener::delimeter_tokenizer("\r"));

  // Method #1:
  //  comparator, callback - async
  FilterPtr f1 =
    listener.createFilter(SerialListener::startsWith("pre"), callback);
  SerialListener::sleep(15); // Sleep 15ms, to let the data come in
  listener.removeFilter(f1); // Not scoped, must be removed explicity

  // Method #2:
  //  comparator - blocking
  {
    BlockingFilterPtr f2 =
      listener.createBlockingFilter(SerialListener::endsWith("post"));
    for (size_t i = 0; i < 3; i++) {
      std::string token = f2->wait(100); // Wait for 100 ms or a matched token
      if (token != "")
        ROS_DEBUG_STREAM( "Found something ending with 'post'" );
      else
        ROS_DEBUG_STREAM( "Did not find something ending with 'post'" );
    }
  }
  // BlockingFilter is scoped and will remove itself, so no removeFilter
  // required, but a call like `listener.removeFilter(BlockingFilter) will
  // remove it from the filter list so wait will always timeout.

  // Method #3:
  //  comparator, token buffer size - blocking
  {
    // Give it a comparator, then a buffer size of 10
    BufferedFilterPtr f3 =
      listener.createBufferedFilter(SerialListener::contains("substr"), 10);
    SerialListener::sleep(75); // Sleep 75ms, should have about 7
    ROS_DEBUG_STREAM( "Caught " << f3->count());
    ROS_DEBUG_STREAM( " tokens containing 'substr'" );
    for(size_t i = 0; i < 20; ++i) {
      std::string token = f3->wait(5); // Pull message from the buffer
      if (token == "") // If an empty string is returned, a timeout occured
        break;
    }
    f3->clear(); // Empties the buffer
    if (f3->wait(0) == "") // Non-blocking wait
      ROS_DEBUG_STREAM( "We won the race condition!" );
    else
      ROS_DEBUG_STREAM( "We lost the race condition..." );
    // The buffer is circular, so the oldest matches will be dropped first
  }
  // BufferedFilter is scoped and will remove itself just like BlockingFilter.

  // Method #4:
  //  callback - async
  // Gets called if a token doesn't match a filter
  listener.setDefaultHandler(default_handler);
  SerialListener::sleep(25); // Sleep 25 ms, so some default callbacks occur

  return 0;

}

int main(void) {
  try {
    return run();
  } catch (std::exception &e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }
}

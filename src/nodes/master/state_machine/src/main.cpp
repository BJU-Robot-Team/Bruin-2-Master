

//main.cpp The Bruin-2 State Machine
#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"
#include "state_machine/ros_interface.h"
#include "ros/ros.h"


#include "sensor_msgs/NavSatFix.h" //for gps callback

#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>

#define STEER_OFFSET 0.02 // Correct for feedback pot not being perfectly mounted

// Relay assignments since we generate relay messages (should be in the relay header file?)
        #define FLASHING_LIGHT 0x8000
        #define FORWARD_RELAY  0x0004
        #define REVERSE_RELAY  0x0008
        #define START_RELAY    0x0080

    roboteq_msgs::Command steerMessage;
    roboteq_msgs::Command brakeMessage;
    digipot::DigipotDataMsg speedMessage;
    relay_board::RelayCommandMsg relayMessage;
    relay_board::RelayDataMsg relayDataMessage;
    state_machine::MsgsForGUI StateOfRobotMessage;
    VehicleData* vehicle_data;

// Callback methods execute on each message receipt,
// load data from messages into vehicle_data
void relay_callback(const relay_board::RelayDataMsg& relayStatusMessage) {
}

void compass_callback(const compass::CompassDataMsg& compassMessage) {
    ROS_DEBUG_STREAM( "State Machine: Compass heading: " << compassMessage.heading << std::endl);
    vehicle_data->position_heading = compassMessage.heading;
}

void camera_callback(const camera_node::CameraDataMsg& cameraMessage) {
    ROS_DEBUG_STREAM("Camera direction: " << cameraMessage.direction << " distance:	 " << cameraMessage.distance << "valid:" << cameraMessage.tracking );
    vehicle_data->follow_direction = cameraMessage.direction;
    vehicle_data->follow_distance = cameraMessage.distance;
    vehicle_data->follow_valid = cameraMessage.tracking;
}

void gps_callback(const sensor_msgs::NavSatFix& gpsMessage) {
    ROS_DEBUG_STREAM( "State Machine: GPS Fix: latitude: " << gpsMessage.latitude << ", longitude: " << gpsMessage.longitude << std::endl);
    vehicle_data->position_latitude = gpsMessage.latitude;
    vehicle_data->position_longitude = gpsMessage.longitude;
}

//TODO: make the GUI adjust which waypoint we go to  
void gui_callback(const master_gui::GUImsg& guiMessage) {
    ROS_DEBUG_STREAM("GUI station selected: " << guiMessage.state << std::endl);
    vehicle_data->selected_station = guiMessage.state;
    //this isn't necessary but I thought a bool was cleaner than using a string and less prone to error
    if (guiMessage.goToNextState == "True") {
        vehicle_data->goto_button_pressed = true;
    } else {
        vehicle_data-> goto_button_pressed = false;
    }



}

int main(int argc, char **argv) {

    VehicleStates last_state;
    bool previous_entry=false, previous_exit=false;  // Joystick entry/exit flags to show/hide window

    //function defined in ros_interface header
    startROS(argc, argv);

    //setup ros interface, state machine, and vehicle data  objects
    ROSInterface ros_interface( &relay_callback, &compass_callback, &camera_callback, &gps_callback, &gui_callback);
    StateMachine state_machine;
    vehicle_data =  new VehicleData();


    state_machine.debug_mode = false;
    bool turn_off_light = false;
    int light_count = 0;

    ROS_INFO_STREAM( "Bruin-2 State Machine Running.");

#define USE_SDL // uncomment this to open and use the SDL window for keyboard input
#ifdef USE_SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) throw std::runtime_error("Could not init SDL");
    SDL_Event event;
    char code;
    
    SDL_Window *window;
    window = SDL_CreateWindow("Bruin-2 Keyboard input",
                          900, 100,
                          215, 298,
                          SDL_WINDOW_OPENGL);
    if ( window == NULL ) {
        ROS_ERROR_STREAM("State Machine: could not open window");
        throw std::runtime_error("State Machine Could not open window");
    }
    SDL_Surface *jpanel, *fpanel;
    jpanel = SDL_LoadBMP("data/j_panel.bmp");  // Bitmap is 215x298
    fpanel = SDL_LoadBMP("data/f_panel.bmp");
    if ( jpanel == NULL ) {
        ROS_ERROR_STREAM("State Machine: could not load j_panel.bmp");
        throw std::runtime_error("State Machine Could not load jpanel bitmap");
    }
    if ( fpanel == NULL ) {
        ROS_ERROR_STREAM("State Machine: could not load f_panel.bmp");
        throw std::runtime_error("State Machine Could not load fpanel bitmap");
    }
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);

    SDL_Texture * jtexture = SDL_CreateTextureFromSurface(renderer, jpanel);
    if ( jtexture == NULL ) {
        ROS_ERROR_STREAM("State Machine: could not create j texture");
        throw std::runtime_error("State Machine Could not create j texture");
    }
    SDL_Texture * ftexture = SDL_CreateTextureFromSurface(renderer, fpanel);
    if ( jtexture == NULL ) {
        ROS_ERROR_STREAM("State Machine: could not create f texture");
        throw std::runtime_error("State Machine Could not create f texture");
    }
    SDL_RenderCopy(renderer, jtexture, NULL, NULL);
    SDL_RenderPresent(renderer);

#endif


    //start main loop
    while (ros_interface.isNodeRunning()) {

#ifdef USE_SDL
        while (SDL_PollEvent(&event)) {
        ROS_DEBUG_STREAM( "got event" << event.type << std::endl);
          switch(event.type) {
          case SDL_KEYUP:
            code = event.key.keysym.sym;
            ROS_DEBUG_STREAM( "got keydown " << code << std::endl);
          break;
          case SDL_KEYDOWN:
            code = event.key.keysym.sym;
            ROS_DEBUG_STREAM("got keydown " << code << std::endl);
            vehicle_data->char_input = code;
          break;
          case SDL_QUIT:
             ROS_DEBUG_STREAM("got quit" << std::endl);
          break;
          case SDL_MOUSEBUTTONDOWN:
            ROS_DEBUG_STREAM( "Mouse: " << event.button.x << " " << event.button.y );
            if ( event.button.y < 75 && event.button.x < 100 ) {
                ROS_DEBUG_STREAM("click go");
                vehicle_data->char_input = 'g';
            }
            else if ( event.button.y < 75 && event.button.x > 100 ) {
                ROS_DEBUG_STREAM("click stop");
                vehicle_data->char_input = 's';
            }
            else if ( event.button.y > 75 && event.button.y<150 && event.button.x > 0 && event.button.x < 70 ) {
                ROS_DEBUG_STREAM("click left");
                vehicle_data->char_input = 'l';
            }
            else if ( event.button.y > 75 && event.button.y<150 && event.button.x > 70 && event.button.x < 140 ) {
                ROS_DEBUG_STREAM("click center");
                vehicle_data->char_input = 'c';
            }
            else if ( event.button.y > 75 && event.button.y<150 && event.button.x > 140 && event.button.x <210 ) {
                ROS_DEBUG_STREAM("click right");
                vehicle_data->char_input = 'r';
            }
            else if ( event.button.y > 150 && event.button.y<225 && event.button.x > 0 && event.button.x < 70 ) {
                ROS_DEBUG_STREAM("click up brake");
                vehicle_data->char_input = 'u';
            }
            else if ( event.button.y > 150 && event.button.y<225 && event.button.x > 140 && event.button.x <210 ) {
                ROS_DEBUG_STREAM("click brake");
                vehicle_data->char_input = 'b';
            }
            else if ( event.button.y > 225 && event.button.x < 70 ) {
                ROS_DEBUG_STREAM("click joystick");
                vehicle_data->char_input = 'j';
            }
            else if ( event.button.y > 225 && event.button.x > 70 && event.button.x < 140 ) {
                ROS_DEBUG_STREAM("click follow");
                vehicle_data->char_input = 'f';
            }
            else if ( event.button.y > 225 && event.button.x > 140 ) {
                ROS_DEBUG_STREAM("click quit");
                vehicle_data->char_input = 'q';
            }
          break;
          default:
             ROS_DEBUG_STREAM( "not up or down or quit" << std::endl);
          break;
          }
        }
#endif

        //Warrning light code
        //TODO: this should have it's own place not slapped in the middle of the main loop. But I'm out of time.
        //Can be invoked by a ROS Timer
        if (turn_off_light) {
            if (light_count < 10) { //cycles for light to be on
                light_count = light_count+1;
            } else {
                relayMessage.command = "OFF";
                turn_off_light = false;
                light_count = 0;
            }

        } else {

            if (light_count < 10) { //cycles for light to be off
                light_count = light_count+1;
            } else {
                relayMessage.command = "ON";
                turn_off_light = true;
                light_count = 0;
            }
        }	


        ROS_DEBUG_STREAM( "follow valid? " << vehicle_data->follow_valid);

        // Message on every state change
        if ( last_state != state_machine.current_state ) {
            ROS_INFO_STREAM( "Current state: " << state_names[state_machine.current_state] );
            last_state = state_machine.current_state;
            previous_entry = false;
        }

        // Show or hide joystick window when you enter or leave JOYSTICK/FOLLOW mode
        // Also switch menu bitmaps when switching between JOYSTICK and FOLLOW
        // In hihdsight, there is probably an easier way...
        if ( (state_machine.current_state == JOYSTICK) || (state_machine.current_state == FOLLOW) ) {
            if ( ! previous_entry ) {
                //previous_entry = true;
                SDL_ShowWindow(window);
                SDL_RenderClear(renderer);
                if ( state_machine.current_state == JOYSTICK) {
                    SDL_RenderCopy(renderer, jtexture, NULL, NULL);
                } else {
                    SDL_RenderCopy(renderer, ftexture, NULL, NULL);
                }
                SDL_RenderPresent(renderer);
                previous_exit = false;
            }
        }
        else {
            if ( ! previous_exit ) {
                SDL_HideWindow(window);
                previous_exit = true;
                previous_entry = false;
            }
        }
        //Run state machine tick
        state_machine.tick(vehicle_data);

        // Publish all the command messages, between every tick of the states

        relayMessage.device_type = "relay";
        relayMessage.device_number = 0;
        relayMessage.command = "writeall";
        if (vehicle_data->speed_cmd < 0.1) {
            // Faking pot with relays; this is speed 0
            relayMessage.mask = 0x0000 | FORWARD_RELAY;
        } else if (vehicle_data->speed_cmd <= 1) {
           // second fixed speed, FORWARD
            relayMessage.mask = 0x0300 | START_RELAY | FORWARD_RELAY;
        } else if (vehicle_data->speed_cmd <= 2) {
           // third fixed speed, FORWARD
            relayMessage.mask = 0x0500 | START_RELAY | FORWARD_RELAY;
        } else if (vehicle_data->speed_cmd <= 3) {
           // fourth fixed speed, FORWARD
            relayMessage.mask = 0x0900 | START_RELAY | FORWARD_RELAY;
        } else {
           // fifth fixed speed, FORWARD
            relayMessage.mask = 0x1100 | START_RELAY | FORWARD_RELAY;
        }
        // Add the state of the flashing light
        if (!turn_off_light) {
           relayMessage.mask = relayMessage.mask | FLASHING_LIGHT;
        }
        ros_interface.relay_pub.publish(relayMessage);

        steerMessage.mode = 1; // 1=MODE_POSITION, 0=MODE_SPEED	
        brakeMessage.mode = 1;

        steerMessage.setpoint = STEER_OFFSET + vehicle_data->steer_cmd*1.5; // deliberately oversteer
        if (speedMessage.speed < (vehicle_data->speed_cmd - 1)) {
          // Don't drop speed suddenly from high speed to zero
          speedMessage.speed = vehicle_data->speed_cmd - 1;
        } else {
          speedMessage.speed = vehicle_data->speed_cmd;
        }
        brakeMessage.setpoint = vehicle_data->brake_cmd;
 
        StateOfRobotMessage.currentState = state_machine.getCurrentState();
        ros_interface.state_pub.publish(StateOfRobotMessage);
        ros_interface.steer_pub.publish(steerMessage);
        ros_interface.brake_pub.publish(brakeMessage);
        ros_interface.speed_pub.publish(speedMessage);
  
    
     
        //check if we are finished
        if ( vehicle_data->shutdown ) endROS();
        ros::spinOnce();  // Need to spin if we use callbacks, and to let ROS know we are alive?        //
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

#ifdef USE_SDL
    SDL_DestroyTexture(jtexture);
    SDL_DestroyTexture(ftexture);
    SDL_FreeSurface(jpanel);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
#endif

}

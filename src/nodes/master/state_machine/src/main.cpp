

#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"
#include "state_machine/ros_interface.h"

#include <vector>
#include <string>
#include <SDL2/SDL.h>

int main(int argc, char **argv) {

    // ROS command messages
    roboteq_msgs::Command steerMessage;
    roboteq_msgs::Command brakeMessage;
    digipot::DigipotDataMsg speedMessage;
    relay_board::RelayCommandMsg relayMessage;

    VehicleStates last_state;
    bool previous_entry=false, previous_exit=false;  // Joystick entry/exit flags to show/hide window

    //function defined in ros_interface header
    startROS(argc, argv);

    //setup ros interface, state machine, and vehicle data  objects
    ROSInterface ros_interface;
    StateMachine state_machine;
    VehicleData* vehicle_data = new VehicleData();
    
    state_machine.debug_mode = false;
    bool turn_off_light = false;
    int light_count = 0;

    ROS_INFO_STREAM( "Bruin-2 State Machine Running.");

#define USE_SDL // uncomment this to open and use the SDL window for keyboard input
#ifdef USE_SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0) throw std::runtime_error("Could not init SDL");
    SDL_Event event;
    char code;
    
    SDL_Window *window = SDL_CreateWindow("Bruin-2 Keyboard input",
                          0, 500,
                          300, 300,
                          SDL_WINDOW_OPENGL);
    //SDL_Surface *screen = SDL_GetWindowSurface(window);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);

        /* Select the color for drawing. It is set to black here. */
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);

        /* Clear the entire screen to our selected color. */
        SDL_RenderClear(renderer);

        /* Up until now everything was drawn behind the scenes.
           This will show the new contents of the window. */
        SDL_RenderPresent(renderer);
#endif


    //start main loop
    while (ros_interface.isNodeRunning()) {


#ifdef USE_SDL
        if (SDL_PollEvent(&event)) {
        ROS_DEBUG_STREAM( "got event" << event.type << std::endl);
          switch(event.type) {
          case SDL_KEYUP:
            code = event.key.keysym.sym;
            ROS_INFO_STREAM( "got keydown" << code << std::endl);
          break;
          case SDL_KEYDOWN:
            code = event.key.keysym.sym;
            ROS_INFO_STREAM("got keydown" << code << std::endl);
            vehicle_data->char_input = code;
          break;
          case SDL_QUIT:
             ROS_DEBUG_STREAM("got quit" << std::endl);
          break;
          default:
             ROS_DEBUG_STREAM( "not up or down or quit" << std::endl);
          break;
          }
        }
#endif

        //Warrning light code
        //TODO: this should have it's own place not slapped int the middle of the main loop. But I'm out of time.
        //Can be invoked by a ROS Timer
        if (turn_off_light) {
            if (light_count < 50) { //cycles for light to be on
                light_count = light_count+1;
            } else {
                relayMessage.command = "OFF";
                turn_off_light = false;
                light_count = 0;
            }

        } else {

            if (light_count < 50) { //cycles for light to be off
                light_count = light_count+1;
            } else {
                relayMessage.command = "ON";
                turn_off_light = true;
                light_count = 0;
            }
        }	

        //check all ROS topics and update vehicle data object
        ros_interface.pollMessages(vehicle_data);

        if ( last_state != state_machine.current_state ) {
            ROS_INFO_STREAM( "Current state: " << state_names[state_machine.current_state] );
            last_state = state_machine.current_state;
        }
        // Show or hide joystick window when you enter or leave JOYSTICK mode
        if ( (state_machine.current_state == JOYSTICK) || (state_machine.current_state == FOLLOW) ) {
            SDL_RaiseWindow(window);
            if ( ! previous_entry ) {
                previous_entry = true;
                SDL_ShowWindow(window);
                SDL_RenderPresent(renderer);
                previous_exit = false;
            }
        }
        else {
            if ( ! previous_exit) {
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
        ros_interface.relay_pub.publish(relayMessage);

        steerMessage.mode = 1; // 1=MODE_POSITION, 0=MODE_SPEED	
        brakeMessage.mode = 1;

        steerMessage.setpoint = vehicle_data->steer_cmd;
        speedMessage.speed = vehicle_data->speed_cmd;
        brakeMessage.setpoint = vehicle_data->brake_cmd;

        ros_interface.steer_pub.publish(steerMessage);
        ros_interface.brake_pub.publish(brakeMessage);
        ros_interface.speed_pub.publish(speedMessage);

        //check if we are finished
        if ( vehicle_data->shutdown ) endROS();
        ros::spinOnce();  // Need to spin if we use callbacks, and to let ROS know we are alive?
    }

#ifdef USE_SDL
    SDL_DestroyWindow(window);
    SDL_Quit();
#endif

}

//main.cpp The Bruin-2 State Machine

#include "state_machine/state_machine.h"
#include "state_machine/vehicle_data.h"
#include "state_machine/ros_interface.h"
#include "state_machine/globals.h"

#include "ros/ros.h"

#include "sensor_msgs/NavSatFix.h" //for gps callback

#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <thread>
#include <SDL2/SDL.h>



VehicleData* vehicle_data; //extern defined in globals




int main(int argc, char **argv) {

    VehicleStates last_state;
    bool previous_entry=false, previous_exit=false;  // Joystick entry/exit flags to show/hide window

    //function defined in ros_interface header
    startROS(argc, argv);

    //setup ros interface, state machine, and vehicle data  objects
    ROSInterface ros_interface;
    StateMachine state_machine;
    vehicle_data =  new VehicleData();


    state_machine.debug_mode = false;

    ROS_INFO_STREAM("Bruin-2 State Machine Running.");


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
        } else {
            if ( ! previous_exit ) {
                SDL_HideWindow(window);
                previous_exit = true;
                previous_entry = false;
            }
        }

        //Run state machine tick
        state_machine.tick(vehicle_data);

}
#endif
        
        //publish the messages we got in the interface
        ros_interface.publishAllMessages(state_machine.getCurrentState());

        //check if we are finished
        if ( vehicle_data->shutdown ) endROS();
        ros::spinOnce();  // Need to spin if we use callbacks, and to let ROS know we are alive
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

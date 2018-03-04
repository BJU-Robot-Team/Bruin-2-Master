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
    bool previous_entry = false, previous_exit = false; // Joystick entry/exit flags to show/hide window
            
    //function defined in ros_interface header
    startROS(argc, argv);
    
    //setup ros interface, state machine, and vehicle data  objects
    ROSInterface ros_interface;
    StateMachine state_machine;
    vehicle_data = new VehicleData();
    
    state_machine.debug_mode = false;
    
    ROS_INFO_STREAM("Bruin-2 State Machine Running.");
    
//Very bad. debugging waypoint list until we can get waypoint map code working
//Load from file (see csv, waypoint_map.h/cpp)
    vehicle_data->waypoints.push_back(Waypoint2(-3.859289358, 138.2808988));
    vehicle_data->waypoints.push_back(Waypoint2(-4.54051674, 125.7158721));
    vehicle_data->waypoints.push_back(Waypoint2(-4.322523979, 104.625901));
    vehicle_data->waypoints.push_back(Waypoint2(-3.505051121, 98.43605011));
    vehicle_data->waypoints.push_back(Waypoint2(-3.614047503, 102.3649375));
    vehicle_data->waypoints.push_back(Waypoint2(11.89068769, 103.106237));
    vehicle_data->waypoints.push_back(Waypoint2(18.48496874, 104.4405762));
    vehicle_data->waypoints.push_back(Waypoint2(29.05761769, 104.2923162));
    vehicle_data->waypoints.push_back(Waypoint2(31.8642745, 104.8112259));
    vehicle_data->waypoints.push_back(Waypoint2(36.60561707, 108.1470737));
    vehicle_data->waypoints.push_back(Waypoint2(40.77472865, 110.5933621));
    vehicle_data->waypoints.push_back(Waypoint2(46.08830222, 115.1523541));
    vehicle_data->waypoints.push_back(Waypoint2(49.6034355, 119.822541));
    vehicle_data->waypoints.push_back(Waypoint2(49.19469908, 120.8232953));
    vehicle_data->waypoints.push_back(Waypoint2(49.6034355, 125.1598975));
    vehicle_data->waypoints.push_back(Waypoint2(48.37722622, 125.4934823));
    vehicle_data->waypoints.push_back(Waypoint2(45.51607122, 127.2726011));
    vehicle_data->waypoints.push_back(Waypoint2(44.61685108, 128.8293301));
    vehicle_data->waypoints.push_back(Waypoint2(41.83744336, 130.9420337));
    vehicle_data->waypoints.push_back(Waypoint2(38.89454107, 133.8331018));
    vehicle_data->waypoints.push_back(Waypoint2(36.19688065, 135.8346104));
    vehicle_data->waypoints.push_back(Waypoint2(31.2920435, 137.3913394));
    vehicle_data->waypoints.push_back(Waypoint2(25.48798622, 138.8368735));
    vehicle_data->waypoints.push_back(Waypoint2(20.82839093, 139.6152379));
    vehicle_data->waypoints.push_back(Waypoint2(18.1307305, 139.0592633));
    vehicle_data->waypoints.push_back(Waypoint2(16.00530107, 138.8368735));
    vehicle_data->waypoints.push_back(Waypoint2(13.5528825, 138.2808988));
    vehicle_data->waypoints.push_back(Waypoint2(10.6917275, 137.8361191));
    vehicle_data->waypoints.push_back(Waypoint2(7.994067071, 136.2793901));
    vehicle_data->waypoints.push_back(Waypoint2(6.113879499, 134.6114662));
    vehicle_data->waypoints.push_back(Waypoint2(3.007482641, 130.497254));
    vehicle_data->waypoints.push_back(Waypoint2(3.007482641, 127.9397706));
    vehicle_data->waypoints.push_back(Waypoint2(1.69952607, 126.8278214));
    vehicle_data->waypoints.push_back(Waypoint2(-0.01716692968, 124.8263127));
    vehicle_data->waypoints.push_back(Waypoint2(-1.079881645, 122.2688294));
    vehicle_data->waypoints.push_back(Waypoint2(-1.16162893, 121.37927));
    vehicle_data->waypoints.push_back(Waypoint2(-0.1806615019, 118.7105917));
    vehicle_data->waypoints.push_back(Waypoint2(0.2280749269, 116.1531084));
    vehicle_data->waypoints.push_back(Waypoint2(3.252724499, 113.9292099));
    vehicle_data->waypoints.push_back(Waypoint2(4.805922927, 111.8165063));
    vehicle_data->waypoints.push_back(Waypoint2(6.522615928, 111.2605317));
    vehicle_data->waypoints.push_back(Waypoint2(5.70514307, 110.5933621));
    vehicle_data->waypoints.push_back(Waypoint2(7.503583357, 108.7030483));
    vehicle_data->waypoints.push_back(Waypoint2(12.24492593, 108.7030483));
    vehicle_data->waypoints.push_back(Waypoint2(19.76567622, 107.0351244));
    vehicle_data->waypoints.push_back(Waypoint2(25.48798622, 104.5888361));
    vehicle_data->waypoints.push_back(Waypoint2(32.51825279, 103.2544969));
    vehicle_data->waypoints.push_back(Waypoint2(46.00655493, 104.700031));
    vehicle_data->waypoints.push_back(Waypoint2(48.94945722, 105.3672005));
    vehicle_data->waypoints.push_back(Waypoint2(55.73448193, 105.8119802));
    vehicle_data->waypoints.push_back(Waypoint2(62.35601208, 107.702294));
    vehicle_data->waypoints.push_back(Waypoint2(61.62028651, 108.9254382));
    vehicle_data->waypoints.push_back(Waypoint2(64.07270508, 108.7030483));
    vehicle_data->waypoints.push_back(Waypoint2(65.13541979, 106.8497996));
    vehicle_data->current_waypoint = vehicle_data->waypoints.begin();
    
#define USE_SDL // uncomment this to open and use the SDL window for keyboard input
#ifdef USE_SDL
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
        throw std::runtime_error("Could not init SDL");
    SDL_Event event;
    char code;
    
    SDL_Window *window;
    window = SDL_CreateWindow("Bruin-2 Keyboard input", 900, 100, 215, 298,
            SDL_WINDOW_OPENGL);
    if (window == NULL) {
        ROS_ERROR_STREAM("State Machine: could not open window");
        throw std::runtime_error("State Machine Could not open window");
    }
    SDL_Surface *jpanel, *fpanel;
    jpanel = SDL_LoadBMP("data/j_panel.bmp");  // Bitmap is 215x298
    fpanel = SDL_LoadBMP("data/f_panel.bmp");
    if (jpanel == NULL) {
        ROS_ERROR_STREAM("State Machine: could not load j_panel.bmp");
        throw std::runtime_error("State Machine Could not load jpanel bitmap");
    }
    if (fpanel == NULL) {
        ROS_ERROR_STREAM("State Machine: could not load f_panel.bmp");
        throw std::runtime_error("State Machine Could not load fpanel bitmap");
    }
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, 0);
    
    SDL_Texture * jtexture = SDL_CreateTextureFromSurface(renderer, jpanel);
    if (jtexture == NULL) {
        ROS_ERROR_STREAM("State Machine: could not create j texture");
        throw std::runtime_error("State Machine Could not create j texture");
    }
    SDL_Texture * ftexture = SDL_CreateTextureFromSurface(renderer, fpanel);
    if (jtexture == NULL) {
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
            ROS_DEBUG_STREAM("got event" << event.type << std::endl);
            switch (event.type) {
                case SDL_KEYUP:
                    code = event.key.keysym.sym;
                    ROS_DEBUG_STREAM("got keydown " << code << std::endl);
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
                    ROS_DEBUG_STREAM(
                            "Mouse: " << event.button.x << " "
                                    << event.button.y);
                    if (event.button.y < 75 && event.button.x < 100) {
                        ROS_DEBUG_STREAM("click go");
                        vehicle_data->char_input = 'g';
                    } else if (event.button.y < 75 && event.button.x > 100) {
                        ROS_DEBUG_STREAM("click stop");
                        vehicle_data->char_input = 's';
                    } else if (event.button.y > 75 && event.button.y < 150
                            && event.button.x > 0 && event.button.x < 70) {
                        ROS_DEBUG_STREAM("click left");
                        vehicle_data->char_input = 'l';
                    } else if (event.button.y > 75 && event.button.y < 150
                            && event.button.x > 70 && event.button.x < 140) {
                        ROS_DEBUG_STREAM("click center");
                        vehicle_data->char_input = 'c';
                    } else if (event.button.y > 75 && event.button.y < 150
                            && event.button.x > 140 && event.button.x < 210) {
                        ROS_DEBUG_STREAM("click right");
                        vehicle_data->char_input = 'r';
                    } else if (event.button.y > 150 && event.button.y < 225
                            && event.button.x > 0 && event.button.x < 70) {
                        ROS_DEBUG_STREAM("click up brake");
                        vehicle_data->char_input = 'u';
                    } else if (event.button.y > 150 && event.button.y < 225
                            && event.button.x > 140 && event.button.x < 210) {
                        ROS_DEBUG_STREAM("click brake");
                        vehicle_data->char_input = 'b';
                    } else if (event.button.y > 225 && event.button.x < 70) {
                        ROS_DEBUG_STREAM("click joystick");
                        vehicle_data->char_input = 'j';
                    } else if (event.button.y > 225 && event.button.x > 70
                            && event.button.x < 140) {
                        ROS_DEBUG_STREAM("click follow");
                        vehicle_data->char_input = 'f';
                    } else if (event.button.y > 225 && event.button.x > 140) {
                        ROS_DEBUG_STREAM("click quit");
                        vehicle_data->char_input = 'q';
                    }
                    break;
                default:
                    ROS_DEBUG_STREAM("not up or down or quit" << std::endl);
                    break;
                    
            }
            
            ROS_DEBUG_STREAM("follow valid? " << vehicle_data->follow_valid);
            
            // Message on every state change
            if (last_state != state_machine.current_state) {
                ROS_INFO_STREAM(
                        "Current state: "
                                << state_names[state_machine.current_state]);
                last_state = state_machine.current_state;
                previous_entry = false;
            }
            
            // Show or hide joystick window when you enter or leave JOYSTICK/FOLLOW mode
            // Also switch menu bitmaps when switching between JOYSTICK and FOLLOW
            // In hihdsight, there is probably an easier way...
            if ((state_machine.current_state == JOYSTICK)
                    || (state_machine.current_state == FOLLOW)) {
                if (!previous_entry) {
                    //previous_entry = true;
                    SDL_ShowWindow(window);
                    SDL_RenderClear(renderer);
                    if (state_machine.current_state == JOYSTICK) {
                        SDL_RenderCopy(renderer, jtexture, NULL, NULL);
                    } else {
                        SDL_RenderCopy(renderer, ftexture, NULL, NULL);
                    }
                    SDL_RenderPresent(renderer);
                    previous_exit = false;
                }
            } else {
                if (!previous_exit) {
                    SDL_HideWindow(window);
                    previous_exit = true;
                    previous_entry = false;
                }
            }
            
        }
#endif
        
        //Run state machine tick
        state_machine.tick(vehicle_data);
        std::cout << "Main tick" << std::endl;
        
        //publish the messages we got in the interface
        ros_interface.publishAllMessages(state_machine.getCurrentState());
        
        //check if we are finished
        if (vehicle_data->shutdown) endROS();
        ros::spinOnce(); // Need to spin if we use callbacks, and to let ROS know we are alive
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

#ifndef GLOBALS_H 
#define GLOBALS_H

// Correct for feedback pot not being perfectly mounted
#define STEER_OFFSET 0.02

// Relay assignments since we generate relay messages (should be in the relay header file?)
#define FLASHING_LIGHT 0x8000
#define FORWARD_RELAY  0x0004
#define REVERSE_RELAY  0x0008
#define START_RELAY    0x0080

extern VehicleData* vehicle_data;

#endif

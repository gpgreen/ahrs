#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <inttypes.h>
#include "canaero.h"
#include "bmp085.h"
#include "l3g4200d.h"

// global state enumeration
enum ahrs_state {AHRSINIT, AHRSLISTEN, AHRSACTIVE};

// the global state
extern enum ahrs_state g_state;

// equipment enabled flags
extern int g_accelerometer_enabled;
extern int g_gyros_enabled;
extern int g_static_air_enabled;
extern int g_dynamic_air_enabled;

// the can stack initialization struct
extern canaero_init_t g_ci;

// bmp085 device structures
extern struct bmp085_dev_t g_bmp085_data[2];

// l3g4200d device structure
extern l3g4200d_dev_t g_gyro_dev;

// the cycle time (approx 80hz) in tenth milliseconds
extern uint16_t g_cycle_time;

// error number
extern volatile uint8_t errcode;

// can stack offline
extern void offline(void);

// main error entry
extern void failed(uint8_t errcode);

#endif  // GLOBALS_H_

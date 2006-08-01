#ifndef DEFS_H
#define DEFS_H
#include "common.h"
#include "SerialCom.h"
#include "WheelEncoder.h"
/************************************************************************/
// Physical constants, in meters, radians, seconds (unless otherwise noted)
#define AXLE_LENGTH      	0.555
#define WHEEL_DIAM       	0.3175  /* 12.5 inches */
#define WHEEL_CIRCUM     	(WHEEL_DIAM * M_PI)
#define TICKS_PER_REV    	3600
#define M_PER_TICK       	(WHEEL_CIRCUM / TICKS_PER_REV)
#define MAX_TICS 	    	16777215U
#define MAX_WHEELSPEED   	5.0

// Wheelchair Controller commands
#define RESET 0
#define POWER 1
#define SETMODE 2 		// Auto or Manual mode
#define GETMODE 3 		// Auto or Manual mode
#define GEAR 4
#define HORN 5

// Wheelchair Controller params
#define ON 1
#define OFF 0
#define UP 1
#define DOWN 0
#define AUTO 1
#define MANUAL 0

// other constants
#define SLEEP 50000
#define LATCHDELAY 1000
#define ENCL_DEFAULT_PORT "/dev/ttyUSB1"
#define ENCR_DEFAULT_PORT "/dev/ttyUSB0"
#define SHRD_DEFAULT_PORT "/dev/ttyS0"
#define ENC_DEFAULT_RATE B9600
#define SHRD_DEFAULT_RATE B115200
#define SLEEP_TIME_USEC 100000

// Global variables, used to solve the problem of two interfaces
// sharing a common serial port
SerialCom * SharedSerial;
int WheelChair_subscriptions;
bool Shared_Serial_Initialised;
pthread_mutex_t sslock;

#endif





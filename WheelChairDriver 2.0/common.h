#ifndef COMMON_H_
#define COMMON_H_

#include <iostream>

#define PLAYER_WHEELCHAIR_SOUND_HORN_REQ      ((uint8_t)1)
#define PLAYER_WHEELCHAIR_SET_MODE_REQ        ((uint8_t)2)
#define PLAYER_WHEELCHAIR_INC_GEAR_REQ        ((uint8_t)3)
#define PLAYER_WHEELCHAIR_DEC_GEAR_REQ        ((uint8_t)4)
#define PLAYER_WHEELCHAIR_SET_POWER_REQ       ((uint8_t)5)
#define PLAYER_WHEELCHAIR_GET_MODE_REQ        ((uint8_t)6)
#define PLAYER_WHEELCHAIR_GET_JOYX_REQ        ((uint8_t)7)
#define PLAYER_WHEELCHAIR_GET_JOYY_REQ        ((uint8_t)8)
#define PLAYER_WHEELCHAIR_GET_POWER_REQ       ((uint8_t)9)
#define ON 1
#define OFF 0
#define HORN_SLEEP 150000
#define UP 1 
#define DOWN 0
#define AUTO 1  
#define MANUAL 0
#define INCREMENT 1  
#define DECREMENT 0
/******************* DATA STARTS******************************/
typedef struct player_wheelchair_data
{
 int32_t joyx, joyy;          // X and Y position of the joystick
 int16_t mode, power;        // mode, Auto or Manual; power, On or Off
} __attribute__ ((packed)) player_wheelchair_data_t;
/******************* DATA ENDS*******************************/


/******************* COMMAND STARTS************************/
typedef struct player_wheelchair_speed_cmd
{
   int32_t xpos, ypos;            // X and Y position 
   int32_t xspeed,yspeed;         // Linear And Angular Velocity in m/sec and rad /sec
   int32_t yaw,yawspeed;          // angle and angular velocity in m/sec
   int32_t state,type;            // state (ON,OFF) and type of command (Position Control or Speed Control)
} __attribute__ ((packed)) player_wheelchair_speed_cmd_t;
/******************* COMMAND ENDS************************/


/*******************CONFIG   STARTS************************/
typedef struct player_wheelchair_config
{
  uint8_t request;
  uint16_t value;
} __attribute__ ((packed)) player_wheelchair_config_t;
/*******************CONFIG      ENDS************************/
#endif

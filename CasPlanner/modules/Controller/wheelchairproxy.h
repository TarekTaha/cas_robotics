/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000-2003
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *                      
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Wheelchair.cc Wheelchair Proxy V 2.0 16/06/2005 
 * Last modified by Tarek Taha
 * client-side wheelchair Proxy 
 */

#include <playerclient.h>
#include <netinet/in.h>
#include <string.h>
#include <math.h>
// #define debug

/*****************************************************************************
 ** begin section WheelChairProxy
 *****************************************************************************/

/** The {\tt WheelChairProxy} class is used to read from the UTS CAS wheelchair
    device. It provides an interface to the wheelchairs hand controller.
 */
class WheelChairProxy : public ClientProxy
{

public:
    /** Constructor.
        Leave the access field empty to start unconnected.
    */
    WheelChairProxy (PlayerClient* pc, unsigned short index, unsigned char access = 'c'):
    ClientProxy(pc,PLAYER_OPAQUE_CODE,index,access){ } ///Class Constructor.
    ~WheelChairProxy() { }       /// Class Destructor
    int32_t joyx, joyy;          /// The Values of the X and Y positions of the joystick.
    int16_t mode, power;         /// The status of the controller (auto/manual, on/off).
    int SoundHorn(unsigned int duration);/// Sounds the horn for a specified duration. 
    int SetMode(int mode_to_set);/// Sets the current control Mode (Manula or Auto).
    int GetMode ();              /// Gets the current Control Mode (Manual or Auto)
    int IncrementGear(int gears);/// Increase the current gear on the wheelchair Certain number of gears. 
    int DecrementGear(int gears);/// Decreases the current gear on the wheelchair Certain number of gears. 
    int SetSpeed(double speed, double sidespeed, double turnrate);/// Sets the motor X and Y speed with a turn rate.
    int SetSpeed(double speed, double sidespeed);/// Sets the motor X and Y speed.
    int SetPower(int statetoset);/// Sets the Power of the Controller to ON or OFF. 
    int GetPower();              /// Gets The Status of the Power ON or OFF.
    void FillData (player_msghdr_t hdr, const char* buffer);  /// interface that all proxies must provide.
    void Print ();               /// Print out the current digital input state.
    double JoyX () ;             /// Returns the Joy X position. 
    double JoyY () ;             /// Returns the Joy Y position. 
    char msg[100];
};
/*****************************************************************************
 ** end section
 *****************************************************************************/
int x;
/*****************************************************************************
 **                          Get POWER Request Starts                       **
 *****************************************************************************/
int WheelChairProxy::GetPower()
{
  if(!client)
    return(-1);
player_wheelchair_config_t config;
strcpy(msg,"\n- Sending Get Power Request (Config) to Server ==>");
memset( &config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_GET_POWER_REQ;
if((x=client->Request(m_device_id,(const char*)&config,sizeof(config)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
#ifdef debug
	printf("%s",msg);
#endif
return this->power; // returns the updated Power !!
}
/*****************************************************************************
 **           Get JOYSTICK  X Position Request Starts                       **
 *****************************************************************************/
double WheelChairProxy::JoyX()
{
  if(!client)
    return(-1);
player_wheelchair_config_t config;
strcpy(msg,"\n- Sending Get Joy X Position Request (Config) to Server ==>");
memset( &config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_GET_JOYX_REQ;
if((x=client->Request(m_device_id,(const char*)&config,sizeof(config)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
#ifdef debug
	printf("%s",msg);
#endif

return this->joyx; // returns the updated Xpos !!
}
/*****************************************************************************
 **           Get JOYSTICK  Y Position Request Starts                       **
 *****************************************************************************/
double WheelChairProxy::JoyY()
{
  if(!client)
    return(-1);
player_wheelchair_config_t config;
strcpy(msg,"\n- Sending Get Joy Y Position Request (Config) to Server ==>");
memset( &config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_GET_JOYY_REQ;

if((x=client->Request(m_device_id,(const char*)&config,sizeof(config)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
#ifdef debug
	printf("%s",msg);
#endif

return this->joyy; // returns the updated Ypos !!
}
/*****************************************************************************
 **                      Sound Horn Request Starts                          **
 *****************************************************************************/
int WheelChairProxy::SoundHorn(unsigned int duration)
{
if(!client)
    return(-1);
player_wheelchair_config_t config;
strcpy(msg,"\n- Sending Sound the horn Request (Config) to Server ==>");
/////////////////Sounding The HORN ON////////////////////////
memset( &config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_SOUND_HORN_REQ;
config.value = ON;
if((x=client->Request(m_device_id,(const char*)&config, sizeof(config)))!=0 )
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");

usleep(duration);// SLeeping for a while
/////////////////Sounding The HORN OFF////////////////////////
memset( &config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_SOUND_HORN_REQ;
config.value = OFF;
if((x=client->Request(m_device_id,(const char*)&config, sizeof(config)))!=0 )
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
#ifdef debug
	printf("%s",msg);
#endif

return x ;
}
/*****************************************************************************
 **                      Set Mode   Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::SetMode(int mode_to_set)
{ 
  if(!client)
    return(-1);
player_wheelchair_config_t config;
strcpy(msg,"\n- Sending Set Mode Request (Config) to Server ==>");
memset( &config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_SET_MODE_REQ;
config.value = mode_to_set;
if((x=client->Request(m_device_id,(const char*)&config,sizeof(config)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
#ifdef debug
	printf("%s",msg);
#endif

return x;
}
/*****************************************************************************
 **                      Get Mode   Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::GetMode()
{
  if(!client)
    return(-1);
player_wheelchair_config_t config;
strcpy(msg,"\n- Sending Get Mode Request (Config) to Server ==>");
memset( &config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_GET_MODE_REQ;

if((x=client->Request(m_device_id,(const char*)&config,sizeof(config)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
#ifdef debug
	printf("%s",msg);
#endif
return this->mode;
}
/*****************************************************************************
 **                      Set Speed  Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::SetSpeed(double speed, double sidespeed, double turnrate)
{
  if(!client)
    return(-1);
player_wheelchair_speed_cmd_t cmd;
strcpy(msg,"\n- Sending Set Speed Command to Server ==>");
memset( &cmd, 0, sizeof(cmd) );
if( speed==0 && sidespeed==0 ) sidespeed=0;
cmd.xspeed=HTOPL(speed*1e3); // m/sec
cmd.yspeed=HTOPL(sidespeed*1e3); // rad/sec
cmd.yaw=HTOPL(turnrate); // m/sec
cmd.state=1;
if((x=client->Write(m_device_id,(const char*)&cmd,sizeof(cmd)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
//strcat(msg,"\n- With Values speed=%lf turnrate=%lf",speed,turnrate);
#ifdef debug
	printf("%s",msg);
#endif
return x;
};

int WheelChairProxy::SetSpeed(double speed, double sidespeed)
{
  if(!client)
    return(-1);
player_wheelchair_speed_cmd_t cmd;
strcpy(msg,"\n- Sending Set Speed Command to Server ==>");
memset( &cmd, 0, sizeof(cmd) );
cmd.xspeed=HTOPL(speed*1e3);
cmd.yspeed=HTOPL(sidespeed*1e3);
cmd.yaw=HTOPL(0);
cmd.state=1;
if((x=client->Write(m_device_id,(const char*)&cmd,sizeof(cmd)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
//strcat(msg,"\n- With Values speed=%lf turnrate=%lf",speed,turnrate);
#ifdef debug
	printf("%s",msg);
#endif
return x;
}

/*****************************************************************************
 **                  Increment Gear  Reguest Starts                         **
 *****************************************************************************/
int WheelChairProxy::IncrementGear(int gears)
{
  if(!client)
    return(-1);
if (gears >5 || gears < 1) 
	{
	strcpy(msg,"\n	-->Gears Value can be between 1 and 5 only");
	return -1;
	}
player_wheelchair_config_t config;
strcpy(msg,"\n- Sending Increment Gear Request (Config) to Server ==>");
memset( &config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_INC_GEAR_REQ;
config.value = gears;
if((x=client->Request(m_device_id,(const char*)&config,sizeof(config)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
#ifdef debug
	printf("%s",msg);
#endif
return x;
}
/*****************************************************************************
 **                  Decrement Gear  Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::DecrementGear(int gears)
{
 if(!client)
  return(-1);
if (gears >5 || gears < 1) 
	{
	strcpy(msg,"\n	-->Gears Value can be between 1 and 5 only");
	return -1;
	}
player_wheelchair_config_t config;
strcpy(msg,"\n- Sending Decrement Gear Request (Config) to Server ==>");
memset(&config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_DEC_GEAR_REQ;
config.value = gears;
if ((x=client->Request(m_device_id,(const char*)&config,sizeof(config)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
#ifdef debug
	printf("%s",msg);
#endif
return x;
}
/*****************************************************************************
 **                      Set Power  Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::SetPower(int state_to_set)
{
  if(!client)
  	return(-1);
player_wheelchair_config_t config;
strcpy(msg,"\n- Sending Set Power Request (Config) to Server ==>");
memset( &config, 0, sizeof(config) );
config.request = PLAYER_WHEELCHAIR_SET_POWER_REQ;
config.value = state_to_set;
if((x=client->Request(m_device_id,(const char*)&config,sizeof(config)))!=0)
	strcat(msg,"... Request Failed !!!");
else
	strcat(msg,"... Request Succeded !");
#ifdef debug
	printf("%s",msg);
#endif
return  x;
}
/*****************************************************************************
 **           Fill Data Class/ Used to fill Data as it arrives              **
 *****************************************************************************/

void WheelChairProxy::FillData(player_msghdr_t hdr, const char* buffer)
{
  if(hdr.size != sizeof(player_wheelchair_data_t))
  {
    if(player_debug_level(-1) >= 1)
      fprintf(stderr,"WARNING: expected %d bytes of wheelchair data, but "
              "received %d. Unexpected results may ensue.\n",
              sizeof(player_wheelchair_data_t),hdr.size);
  }
  joyx = PTOHL(((player_wheelchair_data_t*)buffer)->joyx);
  joyy = PTOHL(((player_wheelchair_data_t*)buffer)->joyy);
  mode = PTOHS(((player_wheelchair_data_t*)buffer)->mode);
  power =PTOHS(((player_wheelchair_data_t*)buffer)->power);
}

/*****************************************************************************
 **                      Print Data in Readable Format                      **
 *****************************************************************************/
void WheelChairProxy::Print()
{  JoyX();
   JoyY();
   GetPower();
   GetMode();
  //printf("\n#Wheel Chair(%d:%d) - %c", m_device_id.code, m_device_id.index, access);
  printf("\nJoyx=%10d JoyY=%10d Mode=%10d Power=%10d",this->joyx,this->joyy,this->mode,this->power);
}


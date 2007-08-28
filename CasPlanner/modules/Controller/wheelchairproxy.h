#ifndef WHEELCHAIRPROXY_H_
#define WHEELCHAIRPROXY_H_

#include <libplayerc++/playerc++.h>
#include <libplayerc/playerc.h>

#include "common.h"

using namespace PlayerCc;

class WheelChairProxy : public ClientProxy
{

  private:

    void Subscribe(uint aIndex);
    void Unsubscribe();

    // libplayerc data structure
    playerc_opaque_t *mDevice;
    player_opaque_data_t mData;
  	player_wheelchair_config_t * config;
	uint size;
  public:
    WheelChairProxy(PlayerClient *aPc, uint aIndex=0);
    ~WheelChairProxy();
    /// How long is the data?
    uint GetCount() const 
    { 
    	return GetVar(mDevice->data_count); 
    };
    /// Opaque data
    void GetData(uint8_t* aDest) const
    {
    	return GetVarByRef(mDevice->data, mDevice->data+GetVar(mDevice->data_count),aDest);
    };
    int32_t joyx, joyy;          /// The Values of the X and Y positions of the joystick.
    int16_t mode, power;         /// The status of the controller (auto/manual, on/off).
    void sendCmd(player_opaque_data_t* aData);    /// Send a command
    int soundHorn(unsigned int duration);/// Sounds the horn for a specified duration. 
    int soundHorn();			/// Sounds the horn for 100 msec.
    int setMode(int mode_to_set);/// Sets the current control Mode (Manula or Auto).
    int getMode ();              /// Gets the current Control Mode (Manual or Auto)
    int incrementGear(int gears);/// Increase the current gear on the wheelchair Certain number of gears. 
    int decrementGear(int gears);/// Decreases the current gear on the wheelchair Certain number of gears. 
    int setSpeed(double speed, double sidespeed, double turnrate);/// Sets the motor X and Y speed with a turn rate.
    int setSpeed(double speed, double sidespeed);/// Sets the motor X and Y speed.
    int setPower(int statetoset);/// Sets the Power of the Controller to ON or OFF. 
    int getPower();              /// Gets The Status of the Power ON or OFF.
    double getJoyX () ;             /// Returns the Joy X position. 
    double getJoyY () ;             /// Returns the Joy Y position. 
};

#endif /*WHEELCHAIRPROXY_H_*/

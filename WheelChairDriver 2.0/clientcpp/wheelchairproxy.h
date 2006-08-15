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

  public:
    WheelChairProxy(PlayerClient *aPc, uint aIndex=0);
    ~WheelChairProxy();
    /// How long is the data?
    uint GetCount() const { return GetVar(mDevice->data_count); };
    /// Opaque data
    void GetData(uint8_t* aDest) const
    {
    	return GetVarByRef(mDevice->data, mDevice->data+GetVar(mDevice->data_count),aDest);
    };
    /// Send a command
    void SendCmd(player_opaque_data_t* aData);
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
    double JoyX () ;             /// Returns the Joy X position. 
    double JoyY () ;             /// Returns the Joy Y position. 
};

#endif /*WHEELCHAIRPROXY_H_*/

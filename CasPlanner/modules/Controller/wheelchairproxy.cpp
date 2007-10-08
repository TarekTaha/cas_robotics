/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "wheelchairproxy.h"

#if HAVE_CONFIG_H
  #include "config.h"
#endif

#include <cassert>
#include <sstream> 
#include <iomanip>
#include <iostream>

using namespace std;
using namespace PlayerCc;

WheelChairProxy::WheelChairProxy(PlayerClient *aPc, uint aIndex)
  : ClientProxy(aPc, aIndex),
  mDevice(NULL)
{
  Subscribe(aIndex);
  // how can I get this into the clientproxy.cc?
  // right now, we're dependent on knowing its device type
  mInfo = &(mDevice->info);
  mData.data_count = sizeof(player_wheelchair_config_t);
  config = reinterpret_cast<player_wheelchair_config_t*>(mData.data);
  size = sizeof(mData) - sizeof(mData.data) + mData.data_count;  
}

WheelChairProxy::~WheelChairProxy()
{
  Unsubscribe();
}

void
WheelChairProxy::Subscribe(uint aIndex)
{
  scoped_lock_t lock(mPc->mMutex);
  mDevice = playerc_opaque_create(mClient, aIndex);
  if (NULL==mDevice)
    throw PlayerError("WheelChairProxy::WheelChairProxy()", "could not create");

  if (0 != playerc_opaque_subscribe(mDevice, PLAYER_OPEN_MODE))
    throw PlayerError("WheelChairProxy::WheelChairProxy()", "could not subscribe");
}

void
WheelChairProxy::sendCmd(player_opaque_data_t* aData)
{
  playerc_opaque_cmd(mDevice, aData);
}

void
WheelChairProxy::Unsubscribe()
{
  assert(NULL!=mDevice);
  scoped_lock_t lock(mPc->mMutex);
  playerc_opaque_unsubscribe(mDevice);
  playerc_opaque_destroy(mDevice);
  mDevice = NULL;
};

int WheelChairProxy::getPower()
{
	scoped_lock_t lock(mPc->mMutex);	
	memset(config, 0, sizeof(config) );
	config->request = PLAYER_WHEELCHAIR_GET_POWER_REQ;		
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),reinterpret_cast<void*>(&mData),size) < 0)	
		return -1;
	else
		return config->value; 
}
/*****************************************************************************
 **           Get JOYSTICK  X Position Request Starts                       **
 *****************************************************************************/
double WheelChairProxy::getJoyX()
{
	scoped_lock_t lock(mPc->mMutex);		
	memset( config, 0, sizeof(config) );
	config->request = PLAYER_WHEELCHAIR_GET_JOYX_REQ;
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),reinterpret_cast<void*>(&mData),size) < 0)	
		return -1;
	else
		return config->value; 
}
/*****************************************************************************
 **           Get JOYSTICK  Y Position Request Starts                       **
 *****************************************************************************/
double WheelChairProxy::getJoyY()
{
	scoped_lock_t lock(mPc->mMutex);		
	memset(config, 0, sizeof(config) );
	config->request = PLAYER_WHEELCHAIR_GET_JOYY_REQ;		
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),reinterpret_cast<void*>(&mData),size) < 0)	
		return -1;
	else
		return config->value; 
}
/*****************************************************************************
 **                      Sound Horn Request Starts                          **
 *****************************************************************************/
int WheelChairProxy::soundHorn()
{
	scoped_lock_t lock(mPc->mMutex);		
	config->value = 100; // 100ms
	config->request = PLAYER_WHEELCHAIR_SOUND_HORN_REQ;	
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),reinterpret_cast<void*>(&mData),size) < 0)	
		return -1;
	else
		return 0;
}

/*****************************************************************************
 **                      Sound Horn Request Starts                          **
 *****************************************************************************/
int WheelChairProxy::soundHorn(unsigned int duration)
{
	scoped_lock_t lock(mPc->mMutex);		
	config->value = duration;
	config->request = PLAYER_WHEELCHAIR_SOUND_HORN_REQ;	
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),reinterpret_cast<void*>(&mData),size) < 0)	
		return -1;
	else
		return 0;
}

/*****************************************************************************
 **                      Get Mode   Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::getMode()
{
	scoped_lock_t lock(mPc->mMutex);		
	memset(config, 0, sizeof(config) );
	config->request = PLAYER_WHEELCHAIR_GET_MODE_REQ;
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),reinterpret_cast<void*>(&mData),size) < 0)	
		return -1;
	else
		return config->value; 
}
int WheelChairProxy::incrementGear(int gears)
{
	scoped_lock_t lock(mPc->mMutex);		
	if (gears >5 || gears < 1) 
	{
		printf("\n	-->Gears Value can be between 1 and 5 only");
		return -1;
	}
	config->value = gears;
	config->request = PLAYER_WHEELCHAIR_INC_GEAR_REQ;		
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),0,0) < 0)	
		return -1;
	else
		return 0;
}
/*****************************************************************************
 **                  Decrement Gear  Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::decrementGear(int gears)
{
	scoped_lock_t lock(mPc->mMutex);		
	if (gears >5 || gears < 1) 
	{
		printf("\n	-->Gears Value can be between 1 and 5 only");
		return -1;
	}
	config->value = gears;
	config->request = PLAYER_WHEELCHAIR_DEC_GEAR_REQ;	
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),0,0) < 0)	
		return -1;
	else
		return 0;
}
/*****************************************************************************
 **                      Set Power  Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::setPower(int state_to_set)
{
	scoped_lock_t lock(mPc->mMutex);		
	config->value = state_to_set;
	config->request = PLAYER_WHEELCHAIR_SET_POWER_REQ;
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),0,0) < 0)	
		return -1;
	else
		return 0;
}

/*****************************************************************************
 **                      Set Mode   Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::setMode(int mode_to_set)
{ 	
	scoped_lock_t lock(mPc->mMutex);		
	config->value = mode_to_set;
	config->request = PLAYER_WHEELCHAIR_SET_MODE_REQ;
	if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                             reinterpret_cast<void*>(&mData),0,0) < 0)	
		return -1;
	else
		return 0;
}

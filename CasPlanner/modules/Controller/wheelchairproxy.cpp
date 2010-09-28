#include "wheelchairproxy.h"
#if HAVE_CONFIG_H
  #include "config.h"
#endif

using namespace std;
WheelChairProxy::WheelChairProxy(PlayerClient *aPc, uint aIndex)
  : ClientProxy(aPc, aIndex),
  mDevice(NULL)
{
    Subscribe(aIndex);
    // how can I get this into the clientproxy.cc?
    // right now, we're dependent on knowing its device type
    mInfo = &(mDevice->info);
    mData.data_count = sizeof(player_wheelchair_config_t);
    // This is stupid, the data in the opaque interface should be allocated by the driver
    // For some unknown reason, it's not allocating upon connection and causing a crash
    // This is a solution to overcome the first access to the config structure
    // The rest of the calls to this interface will use the data allocated by the driver
    // throught the opaque interface !
    mData.data = reinterpret_cast<uint8_t*>(&confSource);
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
WheelChairProxy::SendCmd(player_opaque_data_t* aData)
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

int WheelChairProxy::GetPower()
{
    scoped_lock_t lock(mPc->mMutex);
    if(!config)
    {
        throw PlayerError("\nConfig data structure not allocated yet!");
    }
    memset(config, 0, sizeof(config) );
    config->request = PLAYER_WHEELCHAIR_GET_POWER_REQ;
    if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                              reinterpret_cast<void*>(&mData),reinterpret_cast<void**>(&mData)) < 0)
        return -1;
    else
        return config->value;
}
/*****************************************************************************
 **           Get JOYSTICK  X Position Request Starts                       **
 *****************************************************************************/
double WheelChairProxy::JoyX()
{
    scoped_lock_t lock(mPc->mMutex);
    player_wheelchair_data_t *test  = reinterpret_cast<player_wheelchair_data_t*>(mDevice->data);
    return test->joyx;
}
/*****************************************************************************
 **           Get JOYSTICK  Y Position Request Starts                       **
 *****************************************************************************/
double WheelChairProxy::JoyY()
{
    scoped_lock_t lock(mPc->mMutex);
    player_wheelchair_data_t *test  = reinterpret_cast<player_wheelchair_data_t*>(mDevice->data);
    return test->joyy;
}
/*****************************************************************************
 **                      Sound Horn Request Starts                          **
 *****************************************************************************/
int WheelChairProxy::SoundHorn(unsigned int duration)
{
    scoped_lock_t lock(mPc->mMutex);
    if(!config)
    {
        throw PlayerError("\nConfig data structure not allocated yet!");
    }
    memset(config, 0, sizeof(config) );
    config->value = duration;
    config->request = PLAYER_WHEELCHAIR_SOUND_HORN_REQ;
    if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                              reinterpret_cast<void*>(&mData),reinterpret_cast<void**>(&mData)) < 0)
        return -1;
    else
        return 0;
}

/*****************************************************************************
 **                      Get Mode   Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::GetMode()
{
    scoped_lock_t lock(mPc->mMutex);
    if(!config)
    {
        throw PlayerError("\nConfig data structure not allocated yet!");
    }
    memset(config, 0, sizeof(config) );
    config->request = PLAYER_WHEELCHAIR_GET_MODE_REQ;
    if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                              reinterpret_cast<void*>(&mData),reinterpret_cast<void**>(&mData)) < 0)
        return -1;
    else
        return config->value;
}
int WheelChairProxy::IncrementGear(int gears)
{
    scoped_lock_t lock(mPc->mMutex);
    if(!config)
    {
        throw PlayerError("\nConfig data structure not allocated yet!");
    }
    memset(config, 0, sizeof(config) );
    if (gears >5 || gears < 1)
    {
        printf("\n	-->Gears Value can be between 1 and 5 only");
        return -1;
    }
    config->value = gears;
    config->request = PLAYER_WHEELCHAIR_INC_GEAR_REQ;
    if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                              reinterpret_cast<void*>(&mData),0) < 0)
        return -1;
    else
        return 0;
}
/*****************************************************************************
 **                  Decrement Gear  Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::DecrementGear(int gears)
{
    scoped_lock_t lock(mPc->mMutex);
    if(!config)
    {
        throw PlayerError("\nConfig data structure not allocated yet!");
    }
    memset(config, 0, sizeof(config) );
    if (gears >5 || gears < 1)
    {
        printf("\n	-->Gears Value can be between 1 and 5 only");
        return -1;
    }
    config->value = gears;
    config->request = PLAYER_WHEELCHAIR_DEC_GEAR_REQ;
    if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                              reinterpret_cast<void*>(&mData),0) < 0)
        return -1;
    else
        return 0;
}
/*****************************************************************************
 **                      Set Power  Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::SetPower(int state_to_set)
{
    scoped_lock_t lock(mPc->mMutex);
    if(!config)
    {
        throw PlayerError("\nConfig data structure not allocated yet!");
    }
    memset(config, 0, sizeof(config) );
    config->value = state_to_set;
    config->request = PLAYER_WHEELCHAIR_SET_POWER_REQ;
    if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                              reinterpret_cast<void*>(&mData),0) < 0)
        return -1;
    else
        return 0;
}

/*****************************************************************************
 **                      Set Mode   Reguest Starts                          **
 *****************************************************************************/
int WheelChairProxy::SetMode(int mode_to_set)
{ 	
    scoped_lock_t lock(mPc->mMutex);
    if(!config)
    {
        throw PlayerError("\nConfig data structure not allocated yet!");
    }
    memset(config, 0, sizeof(config) );
    config->value = mode_to_set;
    config->request = PLAYER_WHEELCHAIR_SET_MODE_REQ;
    if(playerc_client_request(mDevice->info.client, &mDevice->info,PLAYER_OPAQUE_REQ,
                              reinterpret_cast<void*>(&mData),0) < 0)
        return -1;
    else
        return 0;
}

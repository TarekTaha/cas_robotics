/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000-2003
 *     Andras Szekely, Aaron Skelsey, Kent Williams
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

/*!
 * WheelchairDriver.cc, v2.00 31/07/2006 
 * This is the New Version of Wheelchair Control Driver for Player 2.0.2
 * This Driver support the following Functions:
 * 1- Setting the HORN
 * 2- Checking AND Setting the control mode (Manual or Automatic)
 * 3- Setting the Gear Speed
 * 4- Activating the Motors with a certain Speed and TurnRate
 * 5- Turns the WheelChair ON and OFF
 *           Done By Tarek Taha
 */
#ifdef HAVE_CONFIG_H
  #include "config.h"
#endif
#include <netinet/in.h>
#include <libplayercore/playercore.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include <math.h>
#include <signal.h>
#include <iostream>
#include "defs.h"
//#define UNLOCK 	pthread_mutex_unlock(&sslock);
//#define LOCK    pthread_mutex_lock(&sslock);
#define UNLOCK 	this->Unlock()
#define LOCK    this->Lock()
using namespace std;
/** @addtogroup drivers Drivers */
/** @{ */
/** @defgroup WheelChair Driver

This driver is created to support UTS CAS Wheelchair Robot, but it
can also be modified to support any mobile robot with 2 wheel motors 
and 2 wheel encoders. The computer running this driver will be talking
to the wheelchair control unit through a serial RS232 port and to the
wheel encoders throught two USB Virtual Ports connected to a serial to
USB convertor.

@par Compile-time dependencies

- none

@par Provides

The p2os driver provides the following device interfaces, some of
them named:

- "Position" @ref player_interface_position
  - This interface returns odometry data, and accepts velocity commands.
- "Opaque"   @ref player_wheelchair_plugin_interface
  - This interface supports requests and commands to the wheelchair control unit

@par Supported configuration requests

- "Position" @ref player_interface_position:
  - PLAYER_POSITION_SET_ODOM_REQ
  - PLAYER_POSITION_MOTOR_POWER_REQ
  - PLAYER_POSITION_RESET_ODOM_REQ
  - PLAYER_POSITION_GET_GEOM_REQ
  - PLAYER_POSITION_VELOCITY_MODE_REQ
- @ref player_wheelchair_plugin_interface
  - PLAYER_WHEELCHAIR_SOUND_HORN_REQ
  - PLAYER_WHEELCHAIR_SET_MODE_REQ
  - PLAYER_WHEELCHAIR_INC_GEAR_REQ
  - PLAYER_WHEELCHAIR_DEC_GEAR_REQ
  - PLAYER_WHEELCHAIR_SET_POWER_REQ
  - PLAYER_WHEELCHAIR_GET_MODE_REQ
  - PLAYER_WHEELCHAIR_GET_JOYX_REQ
  - PLAYER_WHEELCHAIR_GET_JOYY_REQ
  - PLAYER_WHEELCHAIR_GET_POWER_REQ

@par Configuration file options

- Left_Encoder_Port (string)
  - Default: "/dev/ttyUSB1"
  - Port used to communicated with the left encoder
- Right_Encoder_Port (string)
  - Default: "/dev/ttyUSB0"
  - Port used to communicated with the Right encoder
- Serial_Port (string)
  - Default: "/dev/ttyS0"
  - Serial port used to communicate with the control unit.
- Encoder_Baud_Rate (int)
  - Default: "B9600"
  - Baud Rate for encoder ports
- Serial_Baud_Rate (int)
  - Default "B115200"
  - Baud Rate for serial port
- geom_pose_x (float)
  - Default "0"
  - Geometrical position X of the Robot
- geom_pose_y (float)
  - Default "0"
  - Geometrical position Y of the Robot
- geom_pose_theta (float)
  - Default "0"
  - Geometrical position Theta of the Robot
- max_xspeed (length)
  - Default: 0.5 m/s
  - Maximum translational velocity
- max_yawspeed (angle)
  - Default: 100 deg/s
  - Maximum rotational velocity
- debug (bool)
  - Defult: 0 
  - Display Debug Messeges
- Log (bool)
  - Default: 0
  - Loggs the Odom Data (x,y,theta,ltics,rtics,lspeed,rspeed)
@par Example 

@verbatim
driver
(
  name "WheelchairDriver"
  provides ["position:0" "opaque:0"]
  plugin "WheelchairDriver.so"
  Left_Encoder_Port "/dev/ttyUSB1"
  Right_Encoder_Port "/dev/ttyUSB0"
  Serial_Port "/dev/ttyS0"
  Encoder_Baud_Rate "B9600"
)
@endverbatim

@par Authors

Tarek Taha
*/
/** @} */

class WheelchairDriver : public Driver
{
	public:
	    // Must implement the following methods.
	    virtual int Setup();
	    virtual int Shutdown();
	    virtual int ProcessMessage(MessageQueue * resp_queue, 
	                               player_msghdr * hdr, 
	                               void * data);
				int WheelChair_Unit_Reset(void);
//   		virtual int Subscribe(player_devaddr_t id);
//    	virtual int Unsubscribe(player_devaddr_t id);
		WheelchairDriver(ConfigFile* cf, int section); 
    private:	
    	virtual void Main();
		int  HandleConfigs(MessageQueue* resp_queue,player_msghdr * hdr,void * data);
		int  HandleCommands(player_msghdr *hdr,void * data);
	 	void RefreshData();     //refreshs and sends data    
		int  ComputeTickDiff(int from, int to); 
		void UpdateOdom(int ltics, int rtics);  // Updates the Odometry 
		int  GetReading(char Channel);          // Gets reading from wheelchair control unit
		int  WCControl(int WCcmd, bool param);  // Enables communication with the control unit through serial port
   		 	void UpdateMotors(double xspeed, double yawspeed); // Updates the speed and orientation of the motors
	// Position interface
	private: 	
	  	player_devaddr_t position_addr;
	  	// Odometric position (meters,meters,radians)
  	    player_position2d_data_t posdata;
		player_devaddr_t opaque_addr;
		player_opaque_data_t w_data;
		player_wheelchair_data_t * wheelchair_data;
	public :        
		FILE *file;
		int x,y;
		struct timeval last_time,last_position_update;
		float PoseX,PoseY,PoseTheta; // The Geometrical Position
		char Left_Encoder_Port[MAX_FILENAME_SIZE];
		char Right_Encoder_Port[MAX_FILENAME_SIZE];
		int mode,power; // Holds the status of the power and the control mode
		int joyx,joyy;  // Holds the value of the X and Y josystick coordinates
		int Rate, opaque_subscriptions, position_subscriptions;
		unsigned char MoveMode;
		int first_ltics,first_rtics,last_ltics, last_rtics;
		double vint,vdem,vset,vact,vdiff,kvp,kvi,kvd,vact_last,vdem_last; // Liner Velocity Control Parameters.
		double wint,wdem,wset,wact,wdiff,kwp,kwi,kwd,wact_last,wdem_last; // Angular Velocity Control Parameters.
		struct timeval lasttime;
		WheelEncoder * LeftEncoder, * RightEncoder;
		char Serial_Port[MAX_FILENAME_SIZE];
		int  Serial_Rate;
	 	SerialCom * Control_Unit_Serial;
		bool log,debug;
		double oldxspeed,oldyaw, last_theta;
	// Velocity Control Thread Stuff
	private:	
		static void * Run_Velocity_Control(void *driver);
	public: 	
		pthread_t velocity_thread_id;
		void Velocity_Controller(void);
		void Start_Velocity_Thread (void);
		void Stop_Velocity_Thread (void);
};

Driver* WheelchairDriver_Init(ConfigFile* cf, int section) // Create and return a new instance of this driver
{
  return (Driver*) new WheelchairDriver(cf, section);
}

void WheelchairDriver_Register(DriverTable* table)
{
  	table->AddDriver("WheelchairDriver", WheelchairDriver_Init);
}

/* need the extern to avoid C++ name-mangling  */
extern "C"
{
  int player_driver_init(DriverTable* table)
  {
    puts("- Initializing Pluggin Driver ==> Wheelchair Control Driver ...");
    WheelchairDriver_Register(table);
    return(0);
  }
}

void WheelchairDriver::Start_Velocity_Thread(void)
{
	pthread_create(&velocity_thread_id, NULL, &Run_Velocity_Control, this);
};

void WheelchairDriver::Stop_Velocity_Thread(void)
{
	void* dummy;
	pthread_cancel(velocity_thread_id);
  	if(pthread_join(velocity_thread_id,&dummy))
    	perror("WheelchairDriver::Stop_Velocity_Thread:pthread_join()");
};

void * WheelchairDriver::Run_Velocity_Control (void *driver)
{
	// block signals that should be handled by the server thread
	#if HAVE_SIGBLOCK
  		sigblock(SIGINT);
  		sigblock(SIGHUP);
  		sigblock(SIGTERM);
  		sigblock(SIGUSR1);
	#endif
	//pthread_cleanup_push(&DummyMainQuit, devicep);// Install a cleanup function
  	((WheelchairDriver*)driver)->Velocity_Controller(); // rUN THE ACTUAL VELOCITY CONTROL FUNCTION
	//pthread_cleanup_pop(1);// Run, the uninstall cleanup function
	return NULL;
};

void  WheelchairDriver :: Velocity_Controller(void)
{
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL); // Synchronously cancelable thread.
	while (1)
	{
		pthread_testcancel(); // Thread cancellation point.
		x=(LeftEncoder->GetTicks()-first_ltics)*-1;  //Left Motor goes counter clock wise
		y=(RightEncoder->GetTicks()-first_rtics);
		UpdateOdom(x,y);				
		vset =  kvp*(vdem_last - vact) + kvi*vint + kvd*vdiff;
		wset =  kwp*(wdem_last - wact) + kwi*wint + kwd*wdiff;
		if (vdem_last == 0) vset=0;
		if (wdem_last == 0) wset=0;
		UpdateMotors(vset,wset);
		usleep(10000);
	}
	pthread_exit(NULL);
}

int WheelchairDriver :: WheelChair_Unit_Reset()
{
	unsigned int retbyte;

	if (Control_Unit_Serial->SendCommand("O0000") != 0) //Reset output and wait response
	{
		PLAYER_ERROR("\n	-->Failed to find Wheelchair Interface Unit");
		return -1;
	}
	else 
		puts("\n- Wheelchair Interface Unit Found and Reseted.");
	Control_Unit_Serial->ReadByte(&retbyte);
	if (Control_Unit_Serial->SendCommand("L0800") != 0) //Set fwd/rev to 2.5v
	{       
		PLAYER_ERROR("\n	-->Failed to set fwd/rev voltage to 2.5V");
		return -1;
	}
	else
		puts("		+ Forward/Reverse Voltage was successfully set to 2.5V");

	if (Control_Unit_Serial->SendCommand("L1800") != 0) //Set left/right to 2.5v
	{       
		PLAYER_ERROR("\n	-->Failed to set l/r voltage to 2.5V");
		return -1;
	}
	else
		puts("		+ Left/Right Voltage was successfully set to 2.5V");
	return 0;
};

WheelchairDriver::WheelchairDriver(ConfigFile* cf, int section)  : Driver(cf, section)
{
	w_data.data_count = sizeof(wheelchair_data);
	wheelchair_data = reinterpret_cast<player_wheelchair_data_t *>(w_data.data);	
	memset(&this->position_addr, 0, sizeof(position_addr));
	memset(&this->opaque_addr,   0, sizeof(position_addr));
	// Adding Opaque/Customed Plugin interface
	if (cf->ReadDeviceAddr(&(this->opaque_addr), section, "provides", PLAYER_OPAQUE_CODE, -1, NULL) == 0)
  	{
		if (this->AddInterface(this->opaque_addr))
	  	{
	    	this->SetError(-1);    
	    	return;
	  	}
  	}  
    // Adding position interface
    // Do we create a robot position interface?
    if(cf->ReadDeviceAddr(&(this->position_addr), section, "provides", PLAYER_POSITION2D_CODE, -1, NULL) == 0)
    {
		if (this->AddInterface(this->position_addr))
	  	{
	    	this->SetError(-1);    
	    	return;
	  	}
  	}
	LeftEncoder = NULL; RightEncoder = NULL;
	strncpy(Left_Encoder_Port ,cf->ReadString(section, "Left_Encoder_Port", ENCL_DEFAULT_PORT), sizeof(Left_Encoder_Port));
	strncpy(Right_Encoder_Port,cf->ReadString(section, "Right_Encoder_Port", ENCR_DEFAULT_PORT), sizeof(Right_Encoder_Port));
	strncpy(this->Serial_Port, cf->ReadString(section, "Serial_Port", SHRD_DEFAULT_PORT), sizeof(Serial_Port));
	this->Rate =		cf->ReadInt(section, "Encoder_Baud_Rate", ENC_DEFAULT_RATE);
	this->Serial_Rate = cf->ReadInt(section, "Serial_Baud_Rate", SHRD_DEFAULT_RATE);
	this->PoseX=		cf->ReadFloat(section,"geom_pose_x",0);
	this->PoseY=		cf->ReadFloat(section,"geom_pose_y",0);
	this->PoseTheta=	cf->ReadFloat(section,"geom_pose_theta",0);
	debug = 			cf->ReadInt(section, "debug",0);
	log =   			cf->ReadInt(section, "log",0);
	return;
}

int WheelchairDriver::Setup()
{
	printf("\n- Setting UP WheelChair Plugin Driver.");
	LeftEncoder  = new WheelEncoder(Left_Encoder_Port, Rate);
	RightEncoder = new WheelEncoder(Right_Encoder_Port, Rate);
	Control_Unit_Serial = new SerialCom(Serial_Port,Serial_Rate);	
	first_ltics=LeftEncoder->GetTicks();
	first_rtics=RightEncoder->GetTicks();
	last_ltics=0;
	last_rtics=0;
	gettimeofday(&lasttime,NULL);
	gettimeofday(&last_position_update,NULL);
	this->posdata.pos.px = this->posdata.pos.py = this->posdata.pos.pa=0;
	opaque_subscriptions = position_subscriptions = 0;
	x = y = 0;
  	oldxspeed = oldyaw=0;
	vint=vdem=vset=vact=vdiff=vact_last=vdem_last=0; // Liner   Velocity Control Parameters Reset
	wint=wdem=wset=wact=wdiff=wact_last=wdem_last=0; // Angular Velocity Control Parameters Reset
	kvp=0.5; kvi=2; kvd=0.02;
	kwp=0.8; kwi=3; kwd=0.015;
	WheelChair_Unit_Reset();
	if(log)
        	file=fopen("odomlog.txt","wb");
	Start_Velocity_Thread();
	fflush(stdout);
  	StartThread();
	return(0);
}

int WheelchairDriver::Shutdown()
{
	puts("\n- Shutting Down WheelChair Driver");
	//WheelChair_Unit_Reset();
	puts("- Shared Serial Port is De-Initialized by Control Driver");
	unsigned int retbyte;
	//Resetting Motor and Joystick Voltages//
	Control_Unit_Serial->Write("O0000\r", 6);
	Control_Unit_Serial->ReadByte(&retbyte);
	Control_Unit_Serial->ReadByte(&retbyte);
	Control_Unit_Serial->Write("L0800\r", 6);
	Control_Unit_Serial->ReadByte(&retbyte);
	Control_Unit_Serial->ReadByte(&retbyte);
	Control_Unit_Serial->Write("L1800\r", 6);
	Control_Unit_Serial->ReadByte(&retbyte);
	Control_Unit_Serial->ReadByte(&retbyte);
	vint=vdem=vset=vact=vdiff=vact_last=vdem_last=0; // Liner   Velocity Control Parameters Reset
	wint=wdem=wset=wact=wdiff=wact_last=wdem_last=0; // Angular Velocity Control Parameters Reset
	delete Control_Unit_Serial;
	delete LeftEncoder;
	delete RightEncoder;
	Control_Unit_Serial=NULL;
	LeftEncoder=NULL;
	RightEncoder=NULL;
	if(log)
		fclose(file);
	// Stop and join the driver thread
	StopThread();
	Stop_Velocity_Thread();
	return(0);
}

//int WheelchairDriver::Subscribe(player_devaddr_t addr)
//{
//	int retval;
//  	// do the subscription
// 	if((retval = Driver::Subscribe(addr)) == 0)
//  	{
//    	// also increment the appropriate subscription counter
//    	if(Device::MatchDeviceAddress(addr, this->position_addr))
//    	{
//  		  	cout<<"\n Subscribing to position Device"; fflush(stdout);
//      		this->position_subscriptions++;
//    	}
//    	else 
//    	if(Device::MatchDeviceAddress(addr, this->opaque_addr))
//  		{
//  		  	cout<<"\n Subscribing to Opaque Device"; fflush(stdout);  			
//      		this->opaque_subscriptions++;
//  		}
//  	}
//  	return(retval);
//}

//int WheelchairDriver::Unsubscribe(player_devaddr_t addr)
//{
//	int retval;
//
//  	// do the unsubscription
//  	if((retval = Driver::Unsubscribe(addr)) == 0)
//  	{
//	    // also decrement the appropriate subscription counter
//	    if(Device::MatchDeviceAddress(addr, this->position_addr))
//	    {
//	    	this->position_subscriptions--;
//	      	assert(this->position_subscriptions >= 0);
//	    }
//	    else if(Device::MatchDeviceAddress(addr, this->opaque_addr))
//	    {
//	      	this->opaque_subscriptions--;
//	      	assert(this->opaque_subscriptions >= 0);
//	    }
//  	}
//
//  return(retval);
//};

// this function will be run in a separate thread
void WheelchairDriver::Main()
{
	struct timespec sleeptime;
 	int last_pos_subscrcount=0,last_opaque_subscrcount=0;
 	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL); // Synchronously cancelable thread.
	while(1) 
	{
	    // Sleep for 1ms (will actually take longer than this).
		sleeptime.tv_sec = 0;
	    sleeptime.tv_nsec = 1000000L;
	    nanosleep(&sleeptime, NULL);		
	    // Thread cancellation point.
		pthread_testcancel();  
		
		// we want to enable the motors if the first client just subscribed to the position device, 
		// and we want to stop and disable the motors if the last client unsubscribed.
	    this->Lock();
	    if((!last_opaque_subscrcount && this->opaque_subscriptions) || (!last_pos_subscrcount && this->position_subscriptions))
	    	WCControl(POWER,ON);
	    else if((last_opaque_subscrcount && !(this->opaque_subscriptions)) || (last_pos_subscrcount && !(this->position_subscriptions)))
	      	WCControl(POWER,OFF);
	    last_opaque_subscrcount = this->opaque_subscriptions;
	    last_pos_subscrcount = this->position_subscriptions;
	    this->Unlock();		
	    
	    this->ProcessMessages();		
		usleep(100000);        // repeat frequency (default to 100 Hz)
		RefreshData();         // Update posdata
	}
	pthread_exit(NULL);
}

// MessageHandler
int WheelchairDriver::ProcessMessage(MessageQueue * resp_queue, player_msghdr * hdr, void * data)
{
  // Look for configuration requests
  if(hdr->type == PLAYER_MSGTYPE_REQ)
  {
  	cout<<"\n Wheelchair Driver Got a Req"; fflush(stdout);
    return(this->HandleConfigs(resp_queue,hdr,data));
  }
  else if(hdr->type == PLAYER_MSGTYPE_CMD)
  {
//	cout<<"\n Wheelchair Driver Got a CMD"; fflush(stdout);
    return(this->HandleCommands(hdr,data));
  }
  else
    return(-1);
};

int WheelchairDriver::HandleCommands(player_msghdr *hdr,void * data)
{
	int retval = -1;
  	double vdem_new,wdem_new;
  	if(Message::MatchMessage(hdr,
                           PLAYER_MSGTYPE_CMD,
                           PLAYER_POSITION2D_CMD_VEL,
                           this->position_addr))
  	{
	    // get and send the latest motor command
	    cout<<"\n\t\t -Command Is  Velocity"; fflush(stdout);
	    player_position2d_cmd_vel_t position_cmd;
	    position_cmd = *(player_position2d_cmd_vel_t*)data;
        vdem_new = position_cmd.vel.px;
  		wdem_new = position_cmd.vel.pa;
		if (vdem_new != vdem_last || wdem_new != wdem_last)
		{
			// Very Dirty Patch here, this has to be modified later on
			if (vdem_new != vdem_last)
				vint=vdiff=vact_last=0; // Liner   Velocity Control Parameters Reset
			if (wdem_new != wdem_last && wdem_last == 0)
				wint=wdiff=wact_last=0; // Angular Velocity Control Parameters Reset
			vdem=vdem_new;
			wdem=wdem_new;
			//if(this->debug)
				printf("\nPosition Interface Command Recieved ==> Xspeed=:%.3f Yaw=:%.3f",vdem,wdem);
			fflush(stdout);
		}
	    retval = 0;
  	}
  	return retval;
}

int WheelchairDriver::HandleConfigs(MessageQueue* resp_queue,player_msghdr * hdr,void * data)
{
	// Handle Position REQ
	// I didn't like the stupid MessageMatch Method
	// check for position config requests
	if(
	   (hdr->type == (uint8_t)PLAYER_MSGTYPE_REQ) && (hdr->addr.host   == position_addr.host)   &&
	   (hdr->addr.robot  == position_addr.robot) &&  (hdr->addr.interf == position_addr.interf) &&
       (hdr->addr.index  == position_addr.index)
       )
    {
		switch (hdr->subtype)
		{
			case PLAYER_POSITION2D_REQ_GET_GEOM:  
				/* Return the robot geometry. */
			  	cout<<"\n\t -Got GEOM Req"; fflush(stdout);
			    if(hdr->size != 0)
			    {
			      PLAYER_WARN("Arg get robot geom is wrong size; ignoring");
			      return(-1);
			    }
			    player_position2d_geom_t geom;
			    /*! Center of Rotation Pose relative to the center of Area
			     */
			    geom.pose.px = 0.3;
			    geom.pose.py = 0.0;
			    geom.pose.pa = 0.0;
			    geom.size.sl = 1.2;
			    geom.size.sw = 0.65;
			    this->Publish(this->position_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK,PLAYER_POSITION2D_REQ_GET_GEOM,
			    			  (void*)&geom, sizeof(geom), NULL);
			    return(0);
				break;
			case PLAYER_POSITION2D_REQ_SET_ODOM: 
			  	cout<<"\n\t -Got Set Odom Req"; fflush(stdout);
			    if(hdr->size != sizeof(player_position2d_set_odom_req_t))
			    {
			      PLAYER_WARN("Arg to odometry set requests wrong size; ignoring");
			      return(-1);
			    }
		    	player_position2d_set_odom_req_t * set_odom_req;
		    	set_odom_req = (player_position2d_set_odom_req_t*) data;
			    this->posdata.pos.px = (int)rint(set_odom_req->pose.px);
				this->posdata.pos.py = (int)rint(set_odom_req->pose.py);
		  		this->posdata.pos.pa = (int)rint(set_odom_req->pose.pa);
				printf("\n Set Odom request Recieved X=%.3f y=%.3f theta=%.3f",this->posdata.pos.px,this->posdata.pos.py,this->posdata.pos.pa);
			    this->Publish(this->position_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_SET_ODOM);
			    return(0);
				break;
			case PLAYER_POSITION2D_REQ_RESET_ODOM:
			  	cout<<"\n\t -Got Reset Odom";	fflush(stdout);
		    	/* reset position to 0,0,0: no args */
			    if(hdr->size != 0)
			    {
			      PLAYER_WARN("Arg to reset position request is wrong size; ignoring");
			      return(-1);
			    }
		        this->posdata.pos.px = 0;
				this->posdata.pos.py = 0;
		   		this->posdata.pos.pa = 0;
				last_ltics = last_rtics=0;
			    this->Publish(this->position_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_RESET_ODOM);
			    return(0);
			case PLAYER_POSITION2D_REQ_MOTOR_POWER:
			    /* motor state change request
			     *   1 = enable motors
			     *   0 = disable motors (default)
			     */
			  	cout<<"\n\t -Got Motor Power Req"; fflush(stdout);     
			    if(hdr->size != sizeof(player_position2d_power_config_t))
			    {
			      PLAYER_WARN("Arg to motor state change request wrong size; ignoring");
			      return(-1);
			    }
			    player_position2d_power_config_t* power_config;
			    power_config = (player_position2d_power_config_t*)data;
				unsigned int retbyte;
				if (power_config->state == 0)
				{
					LOCK;
					Control_Unit_Serial->Write("O0000\r", 6);
					Control_Unit_Serial->ReadByte(&retbyte);
					Control_Unit_Serial->ReadByte(&retbyte);
//					WCControl(POWER,OFF);
					UNLOCK;
				}
				else
				{
					LOCK;
					Control_Unit_Serial->Write("O0000\r", 6);
					Control_Unit_Serial->ReadByte(&retbyte);
					Control_Unit_Serial->ReadByte(&retbyte);
					Control_Unit_Serial->Write("L0800\r", 6);
					Control_Unit_Serial->ReadByte(&retbyte);
					Control_Unit_Serial->ReadByte(&retbyte);
					Control_Unit_Serial->Write("L1800\r", 6);
					Control_Unit_Serial->ReadByte(&retbyte);
					Control_Unit_Serial->ReadByte(&retbyte);
//					WCControl(POWER,ON);
					UNLOCK;
				}
				this->Publish(this->position_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);
		    	return(0);
		        break;
			default:
			  	cout<<"\n\t -Get Unknown Req"; fflush(stdout);
				return -1;
		}
    }
	// check for opaque config requests
	if(
	   (hdr->type == (uint8_t)PLAYER_MSGTYPE_REQ) && (hdr->addr.host   == opaque_addr.host)    &&
	   (hdr->addr.robot  == opaque_addr.robot)    && (hdr->addr.interf == opaque_addr.interf) &&
       (hdr->addr.index  == opaque_addr.index) && (hdr->subtype == (uint8_t)PLAYER_OPAQUE_REQ)
       )
    {
    	player_wheelchair_config_t * config;
    	config = (player_wheelchair_config_t*) data;    	
	    switch(hdr->subtype)
	    {
	     	case PLAYER_WHEELCHAIR_GET_JOYX_REQ:
				printf("Get JOYX Request");
				if(hdr->size != sizeof(player_wheelchair_config_t)) 
				{
					PLAYER_WARN("Request is of wrong size; ignoring");
				    return(-1);
					break;
				}
				this->joyx = GetReading('A');
				return 0;
	     	case PLAYER_WHEELCHAIR_GET_JOYY_REQ:
				printf("Get JOYY Request");
				if(hdr->size != sizeof(player_wheelchair_config_t)) 
				{
					PLAYER_WARN("Request is of wrong size; ignoring");
				    return(-1);
				}
				this->joyy = GetReading('B');
				return 0;	
			case PLAYER_WHEELCHAIR_GET_MODE_REQ:
				printf("Set Mode Request");
				if(hdr->size != sizeof(player_wheelchair_config_t)) 
				{
					PLAYER_WARN("Request is of wrong size; ignoring");
				    return(-1);
				}
				int mm;
				if( (mm = WCControl(GETMODE,0))!=-1 )
					this->mode = mm ;
				return 0;
	    	case PLAYER_WHEELCHAIR_SET_MODE_REQ:
				printf("Set Mode Request");
				if(hdr->size != sizeof(player_wheelchair_config_t)) 
				{
					PLAYER_WARN("Request is of wrong size; ignoring");
				    return(-1);
				}				
				if (WCControl(SETMODE, (bool)config->value)!=-1)
					this->mode = (bool)config->value;
				return 0;
	    	case PLAYER_WHEELCHAIR_SOUND_HORN_REQ:
				printf("Sound Horn Request");
				if(hdr->size != sizeof(player_wheelchair_config_t)) 
				{
					PLAYER_WARN("Request is of wrong size; ignoring");
				    return(-1);
				}
				WCControl(HORN, config->value);
				return 0;
	    	case PLAYER_WHEELCHAIR_INC_GEAR_REQ:
				printf("Increment Gear Request");
				if(hdr->size != sizeof(player_wheelchair_config_t)) 
				{
					PLAYER_WARN("Request is of wrong size; ignoring");
				    return(-1);
				}
				for(int i=0;i<config->value;i++)
					{
						WCControl(GEAR,INCREMENT);
						usleep(LATCHDELAY);
					}
				return 0;
	    	case PLAYER_WHEELCHAIR_DEC_GEAR_REQ:
				printf("Decrement Gear Request");
				if(hdr->size != sizeof(player_wheelchair_config_t)) 
				{
					PLAYER_WARN("Request is of wrong size; ignoring");
				    return(-1);
				}
				for(int i=0;i<config->value;i++)
					{
						WCControl(GEAR,DECREMENT);
						usleep(LATCHDELAY);
					}
				return 0;
	     	case PLAYER_WHEELCHAIR_GET_POWER_REQ:
				printf("Get Power Request");
				if(hdr->size != sizeof(player_wheelchair_config_t)) 
				{
					PLAYER_WARN("Request is of wrong size; ignoring");
				    return(-1);
				}
				this->power = ((GetReading('A') + GetReading('E')) > 1500)?ON:OFF;
				return 0;
			case PLAYER_WHEELCHAIR_SET_POWER_REQ:
				printf("Set Power Request");
				if(hdr->size != sizeof(player_wheelchair_config_t)) 
				{
					PLAYER_WARN("Request is of wrong size; ignoring");
				    return(-1);
				}
				WCControl(POWER, config->value);
				return 0;
			default:
				return -1;				
		}
    }
	return 0;
}

int WheelchairDriver::WCControl(int WCcmd, bool param) 
{
	static unsigned char status;
	char cmd[7];
	unsigned int retbyte, retbyte2;
	bool power = false;
	switch (WCcmd) 
	{
		case POWER:  
				LOCK;
				if ((GetReading('A') + GetReading('E')) > 1500)
					{ 
					power = true;
					this->power=ON;
					}
				else
					this->power=OFF;
				printf("\n	+-->Processing Power Request ");
				if ((param && !power) || (!param && power)) 
				{
					printf("\n	+-->Trying to toggle the power");
					// Toggle wheelchair power on
					if (Control_Unit_Serial->SendCommand("O0008") != 0) 
						{
							PLAYER_ERROR("Failed to set power\n");
							Control_Unit_Serial->SendCommand("O0000");
							return -1;
						}
					usleep(SLEEP);
					if (Control_Unit_Serial->SendCommand("O0000") != 0)
						{
							PLAYER_ERROR("Failed to set power\n");
							return -1;
						}
				}
				UNLOCK;
			break;
		
		case SETMODE:
				LOCK;
				sprintf(cmd, "U8\r");
				Control_Unit_Serial->Write(cmd, 3);
				printf("\n	+-->Processing Set Mode Request !!!");
				Control_Unit_Serial->ReadByte(&retbyte);
				if ((char)retbyte != 'U') 
				{
					printf("\nReturned: .%c.\n",(char)retbyte);
					PLAYER_ERROR("Failed to read auto/man status from Wheelchair Interface Unit\n");
					return -1;
				}
				Control_Unit_Serial->ReadByte(&retbyte);
				Control_Unit_Serial->ReadByte(&retbyte2);
				Control_Unit_Serial->ReadByte(&retbyte);
				Control_Unit_Serial->ReadByte(&retbyte);
				Control_Unit_Serial->ReadByte(&retbyte);
				
				if (((char)retbyte2 == '0' && param) || ((char)retbyte2 != '0' && !param)) 
				{
					// we need to toggle the mode
					if (Control_Unit_Serial->SendCommand("O0020") != 0) 
					{	
						PLAYER_ERROR("Failed to set mode\n");
						Control_Unit_Serial->SendCommand("O0000");
						return -1;
					}
					usleep(LATCHDELAY);
					if (Control_Unit_Serial->SendCommand("O0000") != 0) 
					{
						PLAYER_ERROR("Failed to set mode\n");
						return -1;
					}
				}
				UNLOCK;
				break;
		case GEAR:   
				LOCK;
				printf("\n	+-->Processing Gear Request /");
				if (param) 
				{
					printf(" Going one Gear UP");
					// Gear up
					status |= 0x02;
					sprintf(cmd,"O00%02X", status);
					if(Control_Unit_Serial->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to increment gear\n");
						return -1;
					}
					usleep(SLEEP);
					usleep(SLEEP);
					usleep(SLEEP);
					status &= ~0x02;
					sprintf(cmd,"O00%02X", status);
					if(Control_Unit_Serial->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to increment gear\n");
						return -1;
					}
				}
				else 
				{
					// Gear down
					printf(" Going one Gear Down");
					status |= 0x01;
					sprintf(cmd,"O00%02X", status);
					if(Control_Unit_Serial->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to decrement gear\n");
						return -1;
					}
					usleep(SLEEP);
					usleep(SLEEP);
					usleep(SLEEP);
					status &= ~0x01;
					sprintf(cmd,"O00%02X", status);
					if(Control_Unit_Serial->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to decrement gear\n");
						return -1;
					}
				}
				UNLOCK;
				break;
		
		case HORN:   
				LOCK;
				printf("\n	+-->Processing Horn Request !!!");
				if (param) 
				{
					// Turn horn on
					status |= 0x04;
					sprintf(cmd,"O00%02X", status);
					if(Control_Unit_Serial->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to enable horn\n");
						return -1;
					}
				}
			 	else 
				{
					// Turn horn off
					status &= ~0x04;
					sprintf(cmd,"O00%02X", status);
					if(Control_Unit_Serial->SendCommand(cmd)!=0) 
					{
						PLAYER_ERROR("Failed to enable horn\n");
						return -1;
					}
				}
				UNLOCK;
			break;
		case GETMODE:
				LOCK;
				printf("\n	+-->Processing Get Mode Request ->");
				sprintf(cmd, "U8\r");
				Control_Unit_Serial->Write(cmd, 3);
				Control_Unit_Serial->ReadByte(&retbyte);
				if ((char)retbyte != 'U') 
				{
					PLAYER_ERROR("\n	-->Failed to read auto/man status from Wheelchair Interface Unit\n");
					return -1;
				}
				Control_Unit_Serial->ReadByte(&retbyte);
				Control_Unit_Serial->ReadByte(&retbyte2);
				Control_Unit_Serial->ReadByte(&retbyte);
				Control_Unit_Serial->ReadByte(&retbyte);
				Control_Unit_Serial->ReadByte(&retbyte);
				
				if ((char)retbyte2 == '0')
				{
					printf("Control is  Manual");
					return (int) MANUAL;
				}
				else
				{
					printf("Control is  Automatic");
					return (int) AUTO;	
				}	
				UNLOCK;		
			break;
		default:
			PLAYER_ERROR("Unknown command\n");
			return -1;
	}
	return 0;
	
};

int WheelchairDriver::GetReading(char Channel) 
{
	int Count = 0;
	unsigned int temp;
	int ret;
	unsigned int Position = 0;
		LOCK;
		Control_Unit_Serial->WriteByte(0x55);
		Control_Unit_Serial->WriteByte(Channel);
		Control_Unit_Serial->WriteByte(13);

		temp = 0;
		ret = Control_Unit_Serial->ReadByte(&temp); 
		if (ret > 0) Count++; 
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		if (temp != 0x55)			
			return -1;

		temp = 0;
		ret = Control_Unit_Serial->ReadByte(&temp); 
		if (ret > 0) Count++; 
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		if ((char)temp != Channel)			
			return -1;
		
		temp = 0;
		ret = Control_Unit_Serial->ReadByte(&temp);
		if (ret > 0) Count++;
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		temp -= 48;
		if (temp > 9)
			temp -= 7; 
		temp = temp << 8;
		Position = temp;
	
		temp = 0;
		ret = Control_Unit_Serial->ReadByte(&temp); 
		if (ret > 0) Count++; 
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		temp -= 48;
		if (temp > 9)
			temp -= 7; 
		temp = temp << 4;
		Position |= temp;
	
		temp = 0;
		ret = Control_Unit_Serial->ReadByte(&temp); 
		if (ret > 0) Count++; 
		if (ret < 0) fprintf(stderr,"Wheelchair: error reading data (%d - %s)\n",errno,strerror(errno));
		temp -= 48;
		if (temp > 9)
			temp -= 7;
		Position |= temp;
	
		ret = Control_Unit_Serial->ReadByte(&temp); 
		UNLOCK;
	if (Count == 5)
		return Position;
	else
		return -1;
};

void WheelchairDriver::UpdateOdom(int ltics, int rtics) 
{
	double delta_left, delta_right, theta, mid_d,sum,diff,circular_r,left_speed,right_speed,rotational_speed;
	struct timeval currtime;
	double timediff;
	gettimeofday(&currtime,NULL);
	timediff = (currtime.tv_sec + currtime.tv_usec/1e6) - (last_time.tv_sec + last_time.tv_usec/1e6);
	last_time = currtime;
	delta_left = (ltics - last_ltics) * M_PER_TICK;    //  Distance travelled by left  wheel since last reading
	delta_right =(rtics - last_rtics) * M_PER_TICK;    //  Distance travelled by Right wheel since last reading
	left_speed = delta_left /timediff;
	right_speed= delta_right/timediff;
	sum= delta_right + delta_left;
	diff=delta_right - delta_left;
	theta = NORMALIZE(diff/AXLE_LENGTH);               //  Angle Theta in Radians normalized between -pi and pi
	last_theta=theta;
	rotational_speed=(theta)/timediff;
	/****************************** Control Part added **************************/
	// Linear velocity Part
	vact = (left_speed + right_speed)/2;
	if (debug)
		printf("\n Vact =%.5f Vint=%.5f Vdiff=%.5f",vact,vint,vdiff);
	vdiff= ((vdem - vact) - (vdem_last - vact_last))/timediff;   // Differential Part
	vint+= ((vdem - vact) + (vdem_last - vact_last))*timediff/2; // integral accumulation part
	vact_last = vact;
	vdem_last = vdem;
	// Angular velocity Part
	wact = rotational_speed;
	wdiff= ((wdem - wact) - (wdem_last - wact_last))/timediff;   // Differential Part
	wint+= ((wdem - wact) + (wdem_last - wact_last))*timediff/2; // integral accumulation part
	wact_last = wact;
	wdem_last = wdem;
	/***************************************************************************/
 	mid_d = sum / 2.0;                                       //  Distance travelled by mid point
	if ( diff==0 )//If we are moving straight, a specail case is needed cause our formula will crash since denomenator will be 0
	{
		posdata.pos.px += (delta_left  * cos(posdata.pos.pa)); // in meters
		posdata.pos.py += (delta_right * sin(posdata.pos.pa)); // in meters
		posdata.pos.pa += (theta);   // no need for this in case of straight line -> delta_theta=0 but just for illustration i kept it
	}
	else // we are moving in an arc
	{
	circular_r = mid_d / theta ;                      // Circular Radius of Trajectory of robot's center	
	posdata.pos.px +=  circular_r * (sin(theta + posdata.pos.pa) - sin(posdata.pos.pa) ); // in meters
	posdata.pos.py -=  circular_r * (cos(theta + posdata.pos.pa) - cos(posdata.pos.pa) ); // in meters
	posdata.pos.pa +=  theta;
	}
	posdata.pos.pa = NORMALIZE(posdata.pos.pa); // Normalize the angle to be between -pi and +pi Normalize(z)= atan2(sin(z),cos(z))
	if( debug)
	{
		printf("\nDelta L=%5.4lf Delta R=%5.4lf Sum=%5.4lf Diff=%5.4lf Theta=%5.4lf Mid=%5.4lf Cir_r=%5.4lf", delta_left, delta_right, sum,diff,theta,mid_d,circular_r);
		printf("\nLastL=%d LastR=%d LTicks= %d RTicks= %d LSpeed= %5.2lf m/sec RSpeed= %5.2lf m/sec diff=%lf",last_ltics,last_rtics,ltics,rtics,left_speed,right_speed,timediff);
		printf("\nWHEELCHAIR: pose: %5.2lf,%5.2lf,%5.2lf", posdata.pos.px,posdata.pos.py,RTOD(posdata.pos.pa));
	}
	if(log)
		fprintf(file,"%d %d %.5f %.5f %.5f %.5f ",ltics,rtics,left_speed,right_speed,rotational_speed,timediff);
	last_ltics = ltics;
	last_rtics = rtics;
};

void WheelchairDriver::RefreshData()
{
  	// Write position data//
	struct timeval now;
	double time_since_last_update;
	gettimeofday(&now,NULL);
	time_since_last_update = (now.tv_sec + now.tv_usec/1e6) - (last_position_update.tv_sec + last_position_update.tv_usec/1e6);
  	
  	wheelchair_data->joyx  = this->joyx;
  	wheelchair_data->joyy  = this->joyy;
  	wheelchair_data->mode  = this->mode;
  	wheelchair_data->power = this->power;
  	uint size = sizeof(w_data) - sizeof(w_data.data) + w_data.data_count;
    Publish(this->opaque_addr, NULL,PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_DATA_STATE,reinterpret_cast<void*>(&w_data), size, NULL);  	

	posdata.vel.px = this->vact;
	posdata.vel.py = this->wact;
  	// put odometry data
    Publish(this->position_addr, NULL,PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,(void*)&posdata, sizeof(posdata), NULL);

	last_position_update = now;
}

void WheelchairDriver::UpdateMotors(double xspeed, double yawspeed) 
{
	double yawval, xval;
	char cmd[7];
	if ((xspeed != oldxspeed) || (yawspeed != oldyaw)) 
	{
		oldxspeed = xspeed;
		oldyaw = yawspeed;
		//xspeed= -14217*pow(xspeed,3)-1875.5*pow(xspeed,2) + 5652.2*xspeed + 1948.7;// i got this equation from matlab after modeling the velocity
		//yaw   = 183.44*pow(yaw,3) - 19.158*pow(yaw,2) - 1147.5*yaw +2019.6;// i got this equation from matlab after modeling the velocity
		xval= -14217*xspeed*xspeed*xspeed-1875.5*xspeed*xspeed + 5652.2*xspeed + 1948.7; //trying to save computational time but avoiding pow math function
		yawval   = 183.44*yawspeed*yawspeed*yawspeed - 19.158*yawspeed*yawspeed - 1147.5*yawspeed + 2019.6;

		// Boundary Checking, Joystick Blocks if voltage limits are exceeded

		if (xval > 3200.0) 
			xval=3200.0;
		else if (xval < 850.0  && xspeed!=0.0) 
			xval= 850.0;
		else if (xspeed == 0.0) 
			xval=1987.0;
		
		if (yawval > 3200.0   ) 
			yawval   = 3200.0;
		else if (yawval <  850.0   && yawspeed!=0 ) 
			yawval   = 850.0;
		else if (yawspeed == 0.0 ) 
			yawval = 1994.0;
			
		sprintf(cmd, "L1%03X", (int)yawval);
		Control_Unit_Serial->SendCommand(cmd);
		sprintf(cmd, "L0%03X", (int)xval);
		Control_Unit_Serial->SendCommand(cmd);
		if( debug)
			printf("\n-->Motors Updated with XSpeed=%.2f Yspeed=%.2f",xval,yawval);
		fflush(stdout);
	}
	fflush(stdout);
};




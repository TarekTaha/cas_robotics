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
#include "wheelchair.h"
//#define UNLOCK 	pthread_mutex_unlock(&sslock);
//#define LOCK    pthread_mutex_lock(&sslock);
#define UNLOCK 	//this->Unlock()
#define LOCK    //this->Lock()
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
   		virtual int Subscribe(player_devaddr_t id);
    	virtual int Unsubscribe(player_devaddr_t id);
		WheelchairDriver(ConfigFile* cf, int section); 
    private:	
    	virtual void Main();
		int  HandleConfigs(MessageQueue* resp_queue,player_msghdr * hdr,void * data);
		int  HandleCommands(player_msghdr *hdr,void * data);
	 	void RefreshData();     //refreshs and sends data    
		void UpdateOdom(int ltics, int rtics);  // Updates the Odometry 
	// Position interface
	private: 	
		WheelChair *wheelChair;
	  	player_devaddr_t position_addr;
	  	// Odometric position (meters,meters,radians)
  	    player_position2d_data_t posdata;
		player_devaddr_t opaque_addr;
		player_opaque_data_t w_data;
		player_wheelchair_data_t * wheelchair_data;
	public :        
		FILE *file;
		struct timeval last_time,last_position_update,stall_time;
		float PoseX,PoseY,PoseTheta; // The Geometrical Position;
		int mode,power; // Holds the status of the power and the control mode
		int joyx,joyy;  // Holds the value of the X and Y josystick coordinates
		int opaque_subscriptions, position_subscriptions;
	 	int last_pos_subscrcount, last_opaque_subscrcount;		
		int last_ltics, last_rtics;
		double vint,vdem,vset,vact,vdiff,kvp,kvi,kvd,vact_last,vdem_last; // Liner Velocity Control Parameters.
		double wint,wdem,wset,wact,wdiff,kwp,kwi,kwd,wact_last,wdem_last; // Angular Velocity Control Parameters.
		double vdem_new,wdem_new;
		struct timeval lasttime;
		bool log,debug,driverInitialized,stall;
		double last_theta;
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
	if(wheelChair)
	{
    		delete wheelChair;
		printf("\n WHEEL CHAIR CLASS CLOSED AND CONNECTION LOST !!!");fflush(stdout);
	}
	printf("\n Velocity Control Thread Terminated !!!\n\n");fflush(stdout);
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
	struct timeval currtime;
	double timediff;	
	// Synchronously cancelable thread.
	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL); 
	int x=0,y=0;	
	while (1)
	{
		// Thread cancellation point.
		pthread_testcancel(); 
		x = wheelChair->getLeftTicks();  
		y = wheelChair->getRightTicks();
//		printf("\n Xticks:%d Yticks:%d",x,y);
		UpdateOdom(x,y);				
		vset =  kvp*(vdem_last - vact) + kvi*vint + kvd*vdiff;
		wset =  kwp*(wdem_last - wact) + kwi*wint + kwd*wdiff;
		if((vdem!=0 && vact ==0) || (wdem!=0 && wact==0))
		{
			if(!stall)
			{
				gettimeofday(&stall_time,NULL);
				stall = true;
			}
			gettimeofday(&currtime,NULL);
			timediff = (currtime.tv_sec + currtime.tv_usec/1e6) - (stall_time.tv_sec + stall_time.tv_usec/1e6);
			if(timediff > 1.0) // no responce for more than 1 sec
			{			
				printf("\n Stall DETECTED!!!");
				posdata.stall = 1; 
				wheelChair->sendCommand(POWER,OFF);
				usleep(200000);
				wheelChair->sendCommand(POWER,ON);
				wheelChair->sendCommand(SETMODE,AUTO);
				usleep(200000);
				wheelChair->driveMotors(0,0);
				usleep(700000);
				posdata.stall = 0; 
				// Reset the velocity Control
				vdem_last=0;
				wdem_last=0;
				vint=vdiff=vact_last=0;
				wint=wdiff=wact_last=0;				
				gettimeofday(&currtime,NULL);
				stall = false;
				continue;
			}
		}
		else
		{
			stall= false;
			posdata.stall = 0; 
		}
		if (vdem_last == 0) vset=0;
		if (wdem_last == 0) wset=0;
		wheelChair->driveMotors(vset,wset);
		usleep(10000);
	}
	pthread_exit(NULL);
}

WheelchairDriver::WheelchairDriver(ConfigFile* cf, int section)  : Driver(cf, section)
{
	w_data.data_count = sizeof(player_wheelchair_data_t);
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
	
	this->PoseX=		cf->ReadFloat(section,"geom_pose_x",0);
	this->PoseY=		cf->ReadFloat(section,"geom_pose_y",0);
	this->PoseTheta=	cf->ReadFloat(section,"geom_pose_theta",0);
	debug = 			cf->ReadInt(section, "debug",0);
	log =   			cf->ReadInt(section, "log",0);
	this->posdata.pos.px = 0;
	this->posdata.pos.py = 0;
	this->posdata.pos.pa = 0;	
	// Create an instance of the Wheelchair and initialize it
	wheelChair = new WheelChair(cf,section);
	return;
}

int WheelchairDriver::Setup()
{
	printf("\n- Setting UP WheelChair Plugin Driver.");
	if(!wheelChair)
	{
		cout<<"\nWheelChair Not Initialized Yet !!!";
		exit(1);
	}
	stall = false;
	last_ltics = wheelChair->getLeftTicks();
	last_rtics = wheelChair->getRightTicks();
	gettimeofday(&lasttime,NULL);
	gettimeofday(&last_position_update,NULL);
	this->posdata.pos.px = this->posdata.pos.py = this->posdata.pos.pa=0;
	opaque_subscriptions = position_subscriptions = 0;
	driverInitialized = false;
	vint=vdem=vset=vact=vdiff=vact_last=vdem_last=0; // Liner   Velocity Control Parameters Reset
	wint=wdem=wset=wact=wdiff=wact_last=wdem_last=0; // Angular Velocity Control Parameters Reset
	kvp=0.5; kvi=2; kvd=0.02;
	kwp=0.8; kwi=3; kwd=0.015;
	if(log)
        	file=fopen("odomlog.txt","wb");
	Start_Velocity_Thread();
  	StartThread();
	return(0);
}

int WheelchairDriver::Shutdown()
{
	vint=vdem=vset=vact=vdiff=vact_last=vdem_last=0; // Liner   Velocity Control Parameters Reset
	wint=wdem=wset=wact=wdiff=wact_last=wdem_last=0; // Angular Velocity Control Parameters Reset
	if(log)
		fclose(file);
	// Stop and join the driver thread
	StopThread();
	usleep(100000);
	if(wheelChair)
	{
		wheelChair->sendCommand(POWER,OFF);
		wheelChair->sendCommand(SETMODE,MANUAL);
	}
	usleep(100000);
	Stop_Velocity_Thread();
	return(0);
}

int WheelchairDriver::Subscribe(player_devaddr_t addr)
{
	int retval;
	// do the subscription
	if((retval = Driver::Subscribe(addr)) == 0)
	{	
	// also increment the appropriate subscription counter
	if(Device::MatchDeviceAddress(addr, this->position_addr))
	{
		cout<<"\n Subscribing to position Device"; fflush(stdout);    		  	
		this->position_subscriptions++;
		if ((position_subscriptions == 1) && !driverInitialized)
		{
			driverInitialized = true;
			wheelChair->sendCommand(POWER,ON);
			wheelChair->sendCommand(SETMODE,AUTO);
			wheelChair->driveMotors(0.0f,0.0f);	
			cout <<"\n--->> WheelChair's Power is turned ON, Mode is AUTONOMOUS\n";
			fflush(stdout);      			
		}
	}
	else 
	if(Device::MatchDeviceAddress(addr, this->opaque_addr))
	{
		cout<<"\n Subscribing to Opaque Device"; fflush(stdout);
		last_opaque_subscrcount = this->opaque_subscriptions;	  		  	  			
		this->opaque_subscriptions++;
		if ((opaque_subscriptions == 1) && !driverInitialized)
		{
			driverInitialized = true;
			wheelChair->sendCommand(POWER,ON);
			wheelChair->sendCommand(SETMODE,AUTO);	
			cout <<"\n--->> WheelChair's Power is turned ON, Mode is AUTONOMOUS\n";
			fflush(stdout);      			
		}	
	}
	}
	return(retval);
}

int WheelchairDriver::Unsubscribe(player_devaddr_t addr)
{
	int retval;
	// do the unsubscription
	if((retval = Driver::Unsubscribe(addr)) == 0)
	{
	    // also decrement the appropriate subscription counter
	    if(Device::MatchDeviceAddress(addr, this->position_addr))
	    {
		this->position_subscriptions--;
		cout<<"\n UnSubscribing from a position Device, Subscriptions Left:"<<position_subscriptions; fflush(stdout);
		assert(this->position_subscriptions >= 0);
		if ((position_subscriptions == 0) && (opaque_subscriptions == 0))
		{
			driverInitialized = false;
			wheelChair->driveMotors(0.0f,0.0f);
			wheelChair->sendCommand(SETMODE,MANUAL);
			wheelChair->sendCommand(POWER,OFF);
			cout <<"\n--->> WheelChair's Power is turned OFF, Mode is MANUAL\n";
			fflush(stdout);
		}
	    }
	    else if(Device::MatchDeviceAddress(addr, this->opaque_addr))
	    {
		this->opaque_subscriptions--;
			cout<<"\n UnSubscribing from an opaque Device, Subscriptions Left:"<<opaque_subscriptions; fflush(stdout);	      	
		assert(this->opaque_subscriptions >= 0);
		if ((position_subscriptions == 0) && (opaque_subscriptions == 0))
		{
			driverInitialized = false;
			wheelChair->sendCommand(SETMODE,MANUAL);
			wheelChair->sendCommand(POWER,OFF);
			cout <<"\n--->> WheelChair's Power is turned OFF, Mode is MANUAL\n";
		}	
	    }
	}
	return(retval);
};

// this function will be run in a separate thread
void WheelchairDriver::Main()
{
	struct timespec sleeptime;
 	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL); // Synchronously cancelable thread.
	while(1) 
	{
	    // Sleep for 1ms (will actually take longer than this).
		sleeptime.tv_sec = 0;
		sleeptime.tv_nsec = 1000000L;
		nanosleep(&sleeptime, NULL);
		// Thread cancellation point.
		pthread_testcancel();
		this->ProcessMessages();		
		RefreshData();         // Update posdata
		struct timeval time_before,time_after;
		double time_between;
		gettimeofday(&time_before,NULL);
		this->joyx = wheelChair->getReading('A');
		usleep(20000);        // repeat frequency (default to 100 Hz)
		this->joyy = wheelChair->getReading('B');
		if(debug)
			cout<<"\nJoyX readings="<<this->joyx<<" Joy Y:"<<this->joyy;
		gettimeofday(&time_after,NULL);
		time_between = (time_after.tv_sec + time_after.tv_usec/1e6) - (time_before.tv_sec + time_before.tv_usec/1e6);
		if(debug)
			cout<<"Time took to get a Reading is:"<<time_between*1000;
	}
	pthread_exit(NULL);
}

// MessageHandler
int WheelchairDriver::ProcessMessage(MessageQueue * resp_queue, player_msghdr * hdr, void * data)
{
  // Look for configuration requests
  if(hdr->type == PLAYER_MSGTYPE_REQ)
  {
  	cout<<"\n Wheelchair Driver Got a Req->"; fflush(stdout);
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
  	if(Message::MatchMessage(hdr,
                           PLAYER_MSGTYPE_CMD,
                           PLAYER_POSITION2D_CMD_VEL,
                           this->position_addr))
  	{
	    // get and send the latest motor command
//	    cout<<"\n\t\t - Command Is  Velocity"; fflush(stdout);
		this->Lock();
		player_position2d_cmd_vel_t position_cmd;
		position_cmd = *(player_position2d_cmd_vel_t*)data;
		vdem_new = position_cmd.vel.px;
		wdem_new = position_cmd.vel.pa;
		if (vdem_new != vdem_last || wdem_new != wdem_last)
		{
			// Very Dirty Patch here, this has to be modified later on
			if(vdem_new != vdem_last)
				vint=vdiff=vact_last=0; // Liner   Velocity Control Parameters Reset
			if(wdem_new != wdem_last && wdem_last == 0)
				wint=wdiff=wact_last=0; // Angular Velocity Control Parameters Reset
			vdem=vdem_new;
			wdem=wdem_new;
			if(this->debug)
				printf("\nPosition Interface Command Recieved ==> Xspeed=:%.3f Yaw=:%.3f",vdem,wdem);
			fflush(stdout);
		}
	    retval = 0;
	    this->Unlock();
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
			  	cout<<" Got GEOM Req"; fflush(stdout);
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
			    geom.size.sl = 0.9;
			    geom.size.sw = 0.7;
			    this->Publish(this->position_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK,PLAYER_POSITION2D_REQ_GET_GEOM,
			    			  (void*)&geom, sizeof(geom), NULL);
			    return(0);
				break;
			case PLAYER_POSITION2D_REQ_SET_ODOM: 
			  	cout<<" Got Set Odom Req"; fflush(stdout);
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
				printf(" Set Odom request Recieved X=%.3f y=%.3f theta=%.3f",this->posdata.pos.px,this->posdata.pos.py,this->posdata.pos.pa);
			    this->Publish(this->position_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_SET_ODOM);
			    return(0);
				break;
			case PLAYER_POSITION2D_REQ_RESET_ODOM:
			  	cout<<" Got Reset Odom";	fflush(stdout);
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
			  	cout<<" Got Motor Power Req"; fflush(stdout);     
			    if(hdr->size != sizeof(player_position2d_power_config_t))
			    {
			      PLAYER_WARN("Arg to motor state change request wrong size; ignoring");
			      return(-1);
			    }
			    player_position2d_power_config_t* power_config;
			    power_config = (player_position2d_power_config_t*)data;
//				unsigned int retbyte;
				if (power_config->state == 0)
				{
//					Control_Unit_Serial->Write("O0000\r", 6);
//					Control_Unit_Serial->ReadByte(&retbyte);
//					Control_Unit_Serial->ReadByte(&retbyte);
				}
				else
				{
//					Control_Unit_Serial->Write("O0000\r", 6);
//					Control_Unit_Serial->ReadByte(&retbyte);
//					Control_Unit_Serial->ReadByte(&retbyte);
//					Control_Unit_Serial->Write("L0800\r", 6);
//					Control_Unit_Serial->ReadByte(&retbyte);
//					Control_Unit_Serial->ReadByte(&retbyte);
//					Control_Unit_Serial->Write("L1800\r", 6);
//					Control_Unit_Serial->ReadByte(&retbyte);
//					Control_Unit_Serial->ReadByte(&retbyte);
				}
				this->Publish(this->position_addr, resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_POSITION2D_REQ_MOTOR_POWER);
		    	return(0);
		        break;
			default:
			  	cout<<" Get Unknown Req"; fflush(stdout);
				return -1;
		}
    }
	// check for opaque config requests
	if(
	   (hdr->type == (uint8_t)PLAYER_MSGTYPE_REQ) && (hdr->addr.host   == opaque_addr.host)    &&
	   (hdr->addr.robot  == opaque_addr.robot)    && (hdr->addr.interf == opaque_addr.interf) &&
       (hdr->addr.index  == opaque_addr.index))//  && (hdr->subtype == (uint8_t)PLAYER_OPAQUE_REQ))
    {
    	player_wheelchair_config_t  * config;
    	player_opaque_data_t  mData = * reinterpret_cast<player_opaque_data_t*>(data);
	mData.data_count = 7;//sizeof(player_wheelchair_config_t);
   	config = reinterpret_cast<player_wheelchair_config_t*>(mData.data);
	uint size ;
	size = sizeof(mData) - sizeof(mData.data) + mData.data_count;	    
	if(this->debug)
		cout<<" Opaque Interface Req->";	fflush(stdout);
	    switch(config->request)
	    {
	     	case PLAYER_WHEELCHAIR_GET_JOYX_REQ:
				struct timeval time_before,time_after;
				double time_between;
				gettimeofday(&time_before,NULL);

				this->joyx = wheelChair->getReading('A');
				cout<<"\nJoyX readings="<<this->joyx;

				gettimeofday(&time_after,NULL);
				time_between = (time_after.tv_sec + time_after.tv_usec/1e6) - (time_before.tv_sec + time_before.tv_usec/1e6);
				cout<<"Time took to get a Reading is:"<<time_between*1000;

				config->value = this->joyx;
			    	this->Publish(this->opaque_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_OPAQUE_REQ);    
				return 0;
	     	case PLAYER_WHEELCHAIR_GET_JOYY_REQ:
			  	//cout<<" Got JOYY Req"; fflush(stdout);     				
				this->joyy = wheelChair->getReading('B');
				//cout<<"JoyY readings="<<this->joyy;
				config->value = this->joyy;
				this->Publish(this->opaque_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_OPAQUE_REQ);    
				return 0;	
		case PLAYER_WHEELCHAIR_GET_MODE_REQ:
			  	cout<<" Got GET Mode Req"; fflush(stdout);     				
				int mm;
				if( (mm = wheelChair->sendCommand(GETMODE,0))!=-1 )
					this->mode = mm ;
				config->value = mm;
			    	this->Publish(this->opaque_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_OPAQUE_REQ, reinterpret_cast<void *>(&mData), size, NULL);
				return 0;
	    	case PLAYER_WHEELCHAIR_SET_MODE_REQ:
			  	cout<<" Got Set Mode Req"; fflush(stdout);     				
				if (wheelChair->sendCommand(SETMODE, (bool)config->value)!=-1)
					this->mode = (bool)config->value;
			    this->Publish(this->opaque_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_OPAQUE_REQ);    
			    return 0;
	    	case PLAYER_WHEELCHAIR_SOUND_HORN_REQ:
			  	cout<<" Got Sound Horn Req"; fflush(stdout);     				
				wheelChair->sendCommand(HORN, config->value);
			    this->Publish(this->opaque_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_OPAQUE_REQ);    
				return 0;
	    	case PLAYER_WHEELCHAIR_INC_GEAR_REQ:
			  	cout<<" Got Increment Gear Req"; fflush(stdout);     				
				for(int i=0;i<config->value;i++)
				{
					wheelChair->sendCommand(GEAR,INCREMENT);
					usleep(LATCHDELAY);
				}
			    this->Publish(this->opaque_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_OPAQUE_REQ);				return 0;
	    	case PLAYER_WHEELCHAIR_DEC_GEAR_REQ:
			  	cout<<" Got Decrement Gear Req"; fflush(stdout);     				
				for(int i=0;i<config->value;i++)
				{
					wheelChair->sendCommand(GEAR,DECREMENT);
					usleep(LATCHDELAY);
				}
			    this->Publish(this->opaque_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_OPAQUE_REQ);    				return 0;
	     	case PLAYER_WHEELCHAIR_GET_POWER_REQ:
			  	cout<<" Got Get Power Req"; fflush(stdout);     				
				this->power = ((wheelChair->getReading('A') + wheelChair->getReading('E')) > 1500)?ON:OFF;
			    	this->Publish(this->opaque_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_OPAQUE_REQ,
			    			  reinterpret_cast<void*>(&mData), size, NULL);    
			  	cout<<" ACK reply SENT"; fflush(stdout);     							    			  
				return 0;
		case PLAYER_WHEELCHAIR_SET_POWER_REQ:
			  	cout<<" Got Set Power Req"; fflush(stdout);     				
				wheelChair->sendCommand(POWER, config->value);
			    	this->Publish(this->opaque_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK, PLAYER_OPAQUE_REQ);    
			  	cout<<" ACK reply SENT"; fflush(stdout);     							    			  
				return 0;
				break;
		default:
			  	cout<<" Got UNKNOWN Req"; fflush(stdout);     							
				return -1;				
		}
    }
	return -1;
}


void WheelchairDriver::UpdateOdom(int ltics, int rtics) 
{
	double delta_left, delta_right, theta, mid_d,sum,diff,circular_r,left_speed,right_speed,rotational_speed;
	struct timeval currtime;
	double timediff;
	int delta_ticksr,delta_ticksl;
	
	gettimeofday(&currtime,NULL);
	timediff = (currtime.tv_sec + currtime.tv_usec/1e6) - (last_time.tv_sec + last_time.tv_usec/1e6);
	last_time = currtime;
	delta_ticksl = (ltics - last_ltics);
	delta_ticksr = (rtics - last_rtics);
	// Detect an OverFlow of Encoder Buffer
	if (delta_ticksl>1e6)
	{
		if(ltics<1e6)
			delta_ticksl = ltics + (MAX_ENCODER_VALUE - last_ltics);
		else
			delta_ticksl = last_ltics + (MAX_ENCODER_VALUE - ltics);
	}

	delta_ticksl*=-1;	//Left Motor goes counter clock wise

	if (delta_ticksr>1e6)
	{
		if(rtics<1e6)
			delta_ticksr = rtics + (MAX_ENCODER_VALUE - last_rtics);
		else
			delta_ticksr = last_rtics + (MAX_ENCODER_VALUE - rtics);
	}
//	printf("\n DeltaR:%d DeltaL:%d",delta_ticksr,delta_ticksl);
	delta_left  = delta_ticksl* M_PER_TICK;    //  Distance travelled by left  wheel since last reading
	delta_right = delta_ticksr* M_PER_TICK;    //  Distance travelled by Right wheel since last reading
	left_speed = delta_left /timediff;
	right_speed= delta_right/timediff;
	sum= delta_right + delta_left;
	diff=delta_right - delta_left;
	theta = NORMALIZE(diff/AXLE_LENGTH);               //  Angle Theta in Radians normalized between -pi and pi
	last_theta=theta;
	rotational_speed=(theta)/timediff;
	/****************************** Control Part added **************************/
	// Linear velocity Part
	this->Lock();
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
	this->Unlock();
	/***************************************************************************/
//	cout<<"\n Vdem:"<<vdem<<" Vset:"<<vset<<" Wdem:"<<wdem<<" Wset:"<<wset;
//	cout<<"\n Vact:"<<vact<<" Vdiff:"<<vdiff<<" Wact:"<<wact<<" Wdiff:"<<wdiff<<" Time Diff is:"<<timediff;
//	fflush(stdout);
//	if(wdem*wact <0 || vdem*vact<0)
//	{
//		cout<<"\n	--->>> Most Probably SOmething is Wrong !!!";
//		//sleep(3);
//	}
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
	// cout<<"\nData Count:"<<size<<"Joyx:"<<wheelchair_data->joyx<<" JoyY:"<<wheelchair_data->joyy<<" Power:"<<wheelchair_data->power;
    	Publish(this->opaque_addr, NULL,PLAYER_MSGTYPE_DATA, PLAYER_OPAQUE_DATA_STATE,reinterpret_cast<void*>(&w_data), size, NULL);  	

	posdata.vel.px = this->vact;
	posdata.vel.py = this->wact;
  	// put odometry data
    Publish(this->position_addr, NULL,PLAYER_MSGTYPE_DATA, PLAYER_POSITION2D_DATA_STATE,(void*)&posdata, sizeof(posdata), NULL);

	last_position_update = now;
};

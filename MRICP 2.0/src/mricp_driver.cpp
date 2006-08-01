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

/*
 * Mricp.cc, v3.0 15/05/2006 
 * This is a Map Reference ICP plugin Driver for real time Map building and 
 * Localization using Iterative Closest Point laser scan matching and 
 * odom correction. Currently The driver is in stable release stage, more modifications
 * might be added later on.
 *           By Tarek Taha / Centre of Autonomous Systems University of Technology Sydney
 */
#ifdef HAVE_CONFIG_H
  #include "config.h"
#endif
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
#include <fstream>
#include <iostream>
#include <signal.h>
#include <netinet/in.h>
// Player includes
#include <libplayercore/playercore.h>

#include <assert.h>
#include "icp.h"
#include "map.h"
#include "lasermodel.h"
#include "Timer.h"

using namespace std;
using namespace Geom2D;

#define MAP_IDX(mf, i, j) ((mf->map_size) * (j) + (i))// compute linear index for given map coords
#define MAP_VALID(mf, i, j) ((i >= 0) && (i < mf->map_size*2/mf->map_resolution) && (j >= 0) && (j < mf->map_size*2/mf->map_resolution))
#define MAXLASERS 4
#define MAXRANGES 10
/** @addtogroup drivers Drivers */
/** @{ */
/** @defgroup MRICP Driver

This driver is created to support UTS CAS mobile Platforms. This driver can be quite 
usefull in a lot of applications where odom correction or matching laser scans is 
essential. I would appreciate any feedback and recommendations that can lead to improving
the performance of this driver.

@par Compile-time dependencies

- none

@par Provides

The MRICP driver provides the following device interfaces, some of
them named:

- "Position" @ref player_interface_position
  - This interface returns odometry data.
- "Opaque"   @ref player_icp_plugin_interface 
  - This is a dummy interface supporting requests and commands to the ICP / TODO
- "Map"   @ref player_map_interface
  - This interface supports map data requests / TODO

@par Supported configuration requests

- "Position" @ref player_interface_position:
  - PLAYER_POSITION_SET_ODOM_REQ
  - PLAYER_POSITION_RESET_ODOM_REQ
  - PLAYER_POSITION_GET_GEOM_REQ
  - "Map" @ref player_interface_map:
  - PLAYER_POSITION_GET_GEOM_DATA
- @ref player_icp_plugin_interface (not clear now, i will know it later)
  - PLAYER_ICP_X_REQ
  - PLAYER_ICP_Y_REQ


@par Configuration file options

- MAXR (double)
  - Default: "7.8"
  - Maximium Laser Range 
- MINR (double)
  - Default: "0.05"
  - Minimium Laser Range
- period (double)
  - Default: "0.5"
  - Time in sec between scans to be matched.
- map_resolution (double)
  - Default: "0.05"
  - Pixel resolution in meter of the map to be build 
- map_size (double)
  - Default: 20
  - This is defined from the origin to the boundary, so is actually half the size of the 
    map PATCH, not the whole map.
- interpolate (bool)
  - Default "1"
  - 0 - Simple ICP, 1 - ICP with interpolation
- NIT (int)
  - Default "10"
  - Number of iterations for each scan-matching.
- gate1 (float)
  - Default "0.5"
  - 1st data association gate for each point in scan
- gate2 (float)
  - Default "0.05"
  - 2nd data association gate for each point in scan
- debug (bool)
  - Defult: 0 
  - Display Debug Messeges
- Log (bool)
  - Default: 0
  - Loggs the Odom Data (x,y,theta,ltics,rtics,lspeed,rspeed)
- map_path(string)
  - Default: "maps/"
  - Specifies the locations where patches and logs are to be saved
- start_in(int)
  - Default : 2
  - Delay Before starting, unit is in seconds
- robot_id(int)
  - Default : 0
  - The Robot id assigned for map identification
- number_of_laser(int)
  - Default : 1
  - The number of lasers to be used in the scan matching (index starts from 0) all lasers
    should be declared in the requires section
- playerv_debug (bool)
  - Default : 0
  - If enabled, the map occupancy will be represented by +1, 0, -1 (occupied, unoccupied,
    unknown), other wise, the probability will be scaled from 0 - 255
- laserX_ranges tuple of (int) where X is an int 
  - Default : [-90 90]
  - Determines the acceptable laser scan ranges, even number of elements should exist in 
    the tuple, smaller range should be first followed by the larger range.
    You will have to manually assign the value of X:
    eg. in ur configuration file u should have something like this for 2 lasers:
        number_of_lasers 2
        laser0_ranges [-90 -70 -50 -30 -10 90]
        laser1_ranges [-120 120]
    - this represent the following acceptable ranges:
      for Laser:0 [-90 -70] [-50 -30] [-10 90]
      for laser:1 [-120 120]
- use_max_range (float)
  - Default: 0
  - Specify if you want to use ranges more than the max to update empty spaces in
    Occupancy grid map, if it's not zero , then the range specified will be used to
    update the cells within that range only (usefull to reduce the effect of false returns)
- sparse_scans_rate (int)
  - Default: 1
  - Specifies the number of laser scan samples resolution, 1 means all beams, 2 means every
    take one every 2 and so on. (OG will not be affected by this, all samples will be use for OG)
    it reduces the CPU usage.
- use_odom (bool)
  - Default: 0
  - Specify if you want to use the underlying poisition driver's odom in laser scan correction
    the existance of an underlying position driver no longer means that u are using the odom
    automatically since it can be used for passing velocity commands.
- free_space_prob (float) between 0 and 1
  - Default: 0.4
  - The probability to be used for updating the free space , lower values will help in reducing
    the false readings effect.
- map_saving_period (float)
  - Default : 10 sec
  - Determines how long time we should wait before saving the map.
@par Example 

@verbatim
driver
(
  name "MRICP_Driver"
  requires ["position:0" "laser:0"]
  provides ["position:1" "opaque:0" "map:0"]
  plugin "MRICP.so"
  MINR 0.05
  MAXR 3.9
  period 1
  map_resolution 0.6
  map_path maps/
  use_max_range 1
  number_of_lasers 2
  free_space_prob 0.4
  sparse_scans_rate 3
  laser0_ranges [-90 -50 -30 90]
  laser1_ranges [-120 120]
  start_in 1
  interpolate 0
  use_odom 1
  robot_id 1
  NIT 15
  log 1
)
@endverbatim

@par Authors

Tarek Taha
*/
/** @} */
  /////////////////////////////////////////////////////////////
 ///                   MRICP DRIVER Class                  ///
/////////////////////////////////////////////////////////////
typedef struct laser_range
{
	int begins;
	int ends;
} laser_ranges_t;
bool is_file(string fname)
{
	//cout<<fname<<endl;
	struct stat stat_buf;
 	if (stat(fname.c_str(),&stat_buf) != 0) return false;
  	return (stat_buf.st_mode & S_IFMT) == S_IFREG;
}
bool is_directory(string fname)
{
	struct stat stat_buf;
  	if (stat(fname.c_str(),&stat_buf) != 0) return false;
  	return (stat_buf.st_mode & S_IFMT) == S_IFDIR;
}
class MrIcpDriver : public Driver
 {
	// Must implement the following methods.
  	public :	
  			int Setup();
			int Shutdown();
	// Constructor
	public:  	MrIcpDriver(ConfigFile* cf, int section); 
	// Main function for device thread.
    private:	
    		virtual void Main();
       		void CheckConfig(); 	//checks for configuration requests
 			void CheckCommands();   //checks for commands
 		 	void RefreshData();     //refreshs and sends data    
 		 	void AddToMap(vector <Point> points_to_add,Pose p); // Add points to Map
 		 	void ResetMap();		// Reset the Map and empty all the point cloud
 		 	Point TransformToGlobal(Point ,Pose p);
 		 	Pose  TransformToGlobal(Pose ,Pose p);
			Point ConvertToPixel(Point p);
			Point ConvertPixel(Point p);
 		 	Pose GetOdomReading();
 		 	int  InRange(double angle,int laser_index);
 		 	void BuildMap();
 		 	void SetupLaser(int);
			void SetupPositionDriver();
			void GenerateLocalMap(Pose pse);
			void ConnectPatches();
			mapgrid_t ComputeRangeProb(double range,bool);
			void HandleGetMapInfo(void *client, void *request, int len);
			void HandleGetMapData(void *client, void *request, int len);
 		 	vector<Point>  GetLaserSample();
	// Position interface / IN
  	private: 	
  			player_device_id_t     position_in_id;
  		 	player_position_data_t position_in_data;
  			player_position_cmd_t  position_in_cmd;
			player_position_geom_t position_in_geom;
			Driver 				   *position_driver;
	// Position interface / OUT
  	private: 	
  			player_device_id_t     position_out_id;
  		 	player_position_data_t position_out_data;
  			player_position_cmd_t  position_out_cmd;
			player_position_geom_t position_out_geom;
	// Laser interface
  	private: 	
			// Supports 2 Lasers
  			player_device_id_t     laser_id[MAXLASERS];
  		 	player_laser_data_t    laser_data;
  			player_laser_config_t  laser_cfg;
			player_laser_geom_t    laser_geom;
			// Used to communicate with the laser Driver
			Driver                 *laser_driver[MAXLASERS]; 
	// Map interface
  	private:	
  			player_device_id_t 	   map_id;
  		 	player_map_data_t 	   map_data;
			player_map_info_t      map_info;
			Driver                 *map_driver;   // Used to communicate with Map Driver
	// Variables
	public :
			FILE *file,*config_file;
			char * map_path;
			int   robot_id,scan_count,nit,nu,map_number,number_of_lasers,
				  range_count[MAXLASERS],sparse_scans_rate;
			// defines a set of ranges (max 10) for each attached laser
		 	laser_range range[MAXLASERS][MAXRANGES];
			float maxr,minr,period,map_resolution,gate1,gate2,map_size,map_saving_period,
				  use_max_range;
		    float PoseX,PoseY,PoseTheta; // Laser Pose
			double px, py, pa,speed,turn_rate,delta_time,start_in,free_space_prob;
			double sanitycheck_distance, sanitycheck_angle; 
			bool log,debug,interpolate,sample_initialized,
			     playerv_debug,position_in_exists,use_odom,reset_timer,warning_misalign;
			struct timeval last_time[MAXLASERS],current_time,laser_timestamp,position_timestamp,
				   loop_start,loop_end,map_timestamp,last_delta,map_current,map_saved;
			ICP icp;
			Pose laser_pose[MAXLASERS],pose_1,pose_2,delta_pose,global_pose,global_pose_prev,global_diff,relative_pose;
			vector<Point> laser_set_1,laser_set_2,local_map,occ_laser_set,map_points;
			MAP *map;
			Timer delta_t_estimation;
};
Driver* MrIcpDriver_Init(ConfigFile* cf, int section) // Create and return a new instance of this driver
{
  	return ((Driver*) (new MrIcpDriver(cf, section)));
}

void MrIcpDriver_Register(DriverTable* table)
{
  	table->AddDriver("MrIcpDriver", MrIcpDriver_Init);
}
/* need the extern to avoid C++ name-mangling  */
extern "C"
{
  int player_driver_init(DriverTable* table)
  {
    puts("	--->>>Initializing Pluggin Driver ==>  MRICP Driver ...");
    MrIcpDriver_Register(table);
    return(0);
  }
}

MrIcpDriver::MrIcpDriver(ConfigFile* cf, int section)  : Driver(cf, section)
{
	char config_temp[40];
  	this->maxr =             cf->ReadFloat(section,"MAXR",7.8);
  	this->start_in =         cf->ReadFloat(section,"start_in",2);
  	this->minr =             cf->ReadFloat(section,"MINR",0.05);
  	this->period =           cf->ReadFloat(section,"period",0.1);
  	this->map_resolution =   cf->ReadFloat(section,"map_resolution",0.05); // In METERS
  	this->map_size = 	     cf->ReadFloat(section,"map_size",20);         // In METERS
  	this->nit =              cf->ReadInt  (section,"NIT",10);
  	this->robot_id =         cf->ReadInt  (section,"robot_id",0);
  	this->number_of_lasers = cf->ReadInt  (section,"number_of_lasers",1);
  	this->gate1 =            cf->ReadFloat(section,"gate1",0.5);
  	this->gate2 =            cf->ReadFloat(section,"gate2",0.05);
  	this->interpolate =      cf->ReadInt  (section, "interpolate", 1);
  	this->map_path	=(char *)cf->ReadString(section,"map_path","maps/");
	this->debug = 			 cf->ReadInt(section,"debug",0); 
	this->log =   			 cf->ReadInt(section,  "log",0); 
	this->use_odom = 		 cf->ReadInt(section,  "use_odom",0); 
	this->playerv_debug =    cf->ReadInt(section,  "playerv_debug",0);
	this->use_max_range =    cf->ReadFloat(section,  "use_max_range",0);
	this->sparse_scans_rate= cf->ReadInt(section,  "sparse_scans_rate",1);
	this->free_space_prob =  cf->ReadFloat(section,"free_space_prob",0.4);
	this->map_saving_period= cf->ReadFloat(section,"map_saving_period",10);
	this->sanitycheck_distance= cf->ReadFloat(section,"sanitycheck_distance",1);
	this->sanitycheck_angle= cf->ReadFloat(section,"sanitycheck_angle",30);
	if (sparse_scans_rate <= 0 )
	{
		cout <<"\nSparse Scans Rate should be positive integer > 0";
		exit(1);
	}
	if(free_space_prob < 0 || free_space_prob >1)
	{
		cout <<"\nFree space probability should be between 0 and 1";
		exit(1);
	}
	for(int k = 0; k < number_of_lasers ; k++)
	{
		sprintf(config_temp,"%s%d%s","laser",k,"_ranges");
		// Expects you to provide "laserx_ranges" (where x is the laser index) 
		// in the configuration file
		this->range_count[k] =	 cf->GetTupleCount(section,config_temp);
		if ((range_count[k]%2)!=0)
		{
			cout<<"\n ERROR: Number of tuples in the ranges for Laser:"<<k<< "should be even !!!";
			exit(1);
		}
		int index=0;
		for(int i=0;i<range_count[k];i+=2)
		{
			this->range[k][index].begins =	 cf->ReadTupleInt(section,config_temp,i ,-90);
			this->range[k][index].ends   =	 cf->ReadTupleInt(section,config_temp,i+1,90);
			if(this->range[k][index].begins > this->range[k][index].ends)
			{
				cout<<"\n ERROR: the beginning range SHOULd be less than the end range !!!";
				exit(1);				
			}
			cout<<"\n Laser:"<<k<<" Range:"<<index<<" begins:"<<range[k][index].begins;
			cout<<" ends:"<<range[k][index].ends;
			index ++;
		}
		range_count[k]/=2;
	}
  // Adding position interface
  // Do we create a position interface?
  if(cf->ReadDeviceId(&(this->position_out_id), section, "provides", PLAYER_POSITION_CODE, -1, NULL) == 0)
  {
  if (this->AddInterface(this->position_out_id , //1- Supports the Position Interface indicated by it's code "PLAYER_POSITION_CODE"
                                PLAYER_ALL_MODE, //2- Allows Read/Write access from the clients indicated by "PLAYER_ALL_MODE"
		    	 sizeof(player_position_data_t), //3- Has a data buffer big enough to hold a standard position interface data
			                                     // packet sizeof(player_position_data_t)
                  sizeof(player_position_cmd_t), //4- Has a command buffer large enougg to hold the standard position interface 
			                                     //command packet sizeof(player_position_cmd_t)
			                                 10, //5- Can queue upto 10 incoming configuration requests and 10 outgoing
			                                     //configuration replies
			                                10)!= 0) 
    {
      this->SetError(-1);    
      return;
    }
  }
  else
  	{
  		cout<<"\n --->>> ERROR: position out not specified in the configuration FILE";
  		exit(-1);
  	}
  if(this->debug)
	  cout<<"\n Position out Interface Loaded";
  fflush(stdout);
  // Adding MAP interface
  // Do we create a MAP interface?
  if(cf->ReadDeviceId(&(this->map_id), section, "provides", PLAYER_MAP_CODE, -1, NULL) == 0)
  {
   if (this->AddInterface(this->map_id , //1- Supports the Map Interface indicated by it's code "PLAYER_MAP_CODE"
                        PLAYER_ALL_MODE, //2- Allows Read/Write access from the clients indicated by "PLAYER_ALL_MODE"
			  sizeof(player_map_data_t), //3- Has a data buffer big enough to hold a standard MAP interface data
			                             // packet sizeof(player_map_data_t)
                                      0, //4- NO commands for map interface
			                          10,//5- Can queue upto 10 incoming configuration requests and 10 outgoing
			                             //configuration replies
			                          10)!= 0) 
	{
      	this->SetError(-1);    
     	return;
    }
	if(this->debug)
		cout<<"\n MAP Interface Loaded";
  }

  // Adding LASER interfaces
	for(int i=0; i<this->number_of_lasers;i++)
	{
  		if(cf->ReadDeviceId(&(this->laser_id[i]), section, "requires", PLAYER_LASER_CODE,i, NULL) == 0)
  		{
			SetupLaser(i); // Here we initialize the talk to the laser driver
			cout<<"\n LASER Interface Loaded Success index:"<<i; fflush(stdout);
	  		if(this->debug)
				cout<<"\n LASER Interface Loaded";
  		}
  		else
		{
			cout<<"\n Incorrect Number of Lasers Specified, check your config file ...";
			fflush(stdout);
			exit(1);
		}
	}
   // Adding position interface
   // Do we create a position interface?
	if(cf->ReadDeviceId(&(this->position_in_id), section, "requires", PLAYER_POSITION_CODE, -1, NULL) == 0)
  	{
		SetupPositionDriver();
		position_in_exists = true; 
		if(this->debug)
			cout<<"\n Position IN Interface Loaded";
  	}
  	else
  	{
  		cout<<"\n No position:0 is found, ignoring it ...";
  		if (this->use_odom)
  		  		cout<<"\n Can't use odom when you don't have an underlying position driver !!!";
  		position_in_exists = false;
  		this->position_driver = NULL;
  	}
  	cout<<"\n	--->>>Gate1 ="<<gate1<<" Gate2="<<gate2<<" NIT="<<nit<<" MAXR="<<maxr<<" MINR="<<minr<<endl;
  	fflush(stdout);
  	return;
}

int MrIcpDriver::Setup()
{
	char filename[40],command[40];
	printf("\n- Setting UP MRICP Plugin Driver.");
	if(!is_directory(map_path))
	{
		sprintf(command,"%s%s","mkdir ",map_path); 
		if(system(command)==-1)
		{
			perror("\n Serious Error Happened while trying to create the folder");
			exit(1);
		}
		else
			cout<<"\nFolder Created Successfully";
	}
	// allocate space for map cells
  	//assert(this->map_data = (char*)malloc(sizeof(char)*this->map_size*this->map_size));
  	for(int i=0;i<number_of_lasers;i++)
		gettimeofday(&last_time[i],NULL);
	this->global_pose.p.x = this->global_pose.p.y = this->global_pose.phi = 0;
	this->px=0;
	this->py=0;
	this->pa=0;
	this->debug = 0;
	nu = 0; map_number = 1;
	gchar * g_filename=g_strdup_printf("%sMAP_PATCH0",map_path);
	this->map = new MAP(g_filename,this->map_resolution,this->map_size*2);
	this->map->CreateMap();
	this->map->ResetProb();
	sprintf(filename,"%spatch_config.txt",map_path);
	config_file = fopen(filename,"wb");
	// Initial Patch Settings	
	fprintf(config_file,"%s %.3f %.3f %.3f\n","MAP_PATCH0",0.0,0.0,0.0); 
	delta_pose.p.x=0;
	delta_pose.p.y=0;
	delta_pose.phi=0;
	reset_timer= true;
	sample_initialized = FALSE;
	if(log)
	{
        sprintf(filename,"%sicplog.txt",map_path);
		file=fopen(filename,"wb");
	}
	usleep((int)(this->start_in*1e6));
  	this->StartThread();
	return(0);
};
void MrIcpDriver::ResetMap()
{
	char filename[40];
	gchar * savefile;
	sample_initialized = FALSE;
	laser_set_1.clear();
	laser_set_2.clear();
	map_points.clear();
	//map->SavePixelBufferToFile();
	map->SavePgm();
	map->ClearData();
	sprintf(filename,"MAP_PATCH%d",map_number++);
  	savefile=g_strdup_printf("%s%s",map_path,filename);
  	map->Mapname = (char *)realloc(map->Mapname,strlen(savefile)*(sizeof(savefile[0])));
 	strcpy(map->Mapname,savefile);
	this->map->ResetProb();
	// Save Patch's Name + origin in terms of the previous Patch
	fprintf(config_file,"%s %.3f %.3f %.3f\n",filename,global_pose.p.x,global_pose.p.y,global_pose.phi);
	this->global_pose.p.x = this->global_pose.p.y = this->global_pose.phi = 0;
}
int MrIcpDriver::Shutdown()
{
	// Stop and join the driver thread
	cout<<"\n- Shutting Down MRICP Driver - Cleaning up Mess ..\n"; fflush(stdout);
	for(int i=0;i<number_of_lasers;i++)
	{
    	this->laser_driver[i]->Unsubscribe(this->laser_id[i]);
    	this->laser_driver[i] = NULL;
	}
    //this->map->SavePixelBufferToFile();
    this->map->SavePgm();
    delete this->map;
	cout<<"\n	--->>> MAP Buffer Deleted->"; fflush(stdout);
	this->occ_laser_set.clear();
    this->map_points.clear();
    this->laser_set_1.clear();
    this->laser_set_2.clear();
    this->local_map.clear();
	cout<<" Vectors Cleared ->"; fflush(stdout);
	if(log)
		fclose(file);
	fclose(config_file);
	cout<<" Files Closed ->"; fflush(stdout);
	//ConnectPatches();
  	this->StopThread();
	cout<<" Thread Killed ->"; fflush(stdout);
	cout<<" ... ShutDown FINISED\n"; fflush(stdout);
	return(0);
}; 
// this function will run in a separate thread
void MrIcpDriver::Main()
{
	Timer loop_timer,map_timer,test;
	double time_elapsed;
	// Synchronously cancelable thread.
	//pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);
	
	/* To use the Timer.Synch() in a loop you will have to 
	 * reset the timer at the beginning of the loop then call
	 * the Timer.Synch(period) at the end of the loop with 
	 * period representing the synchronization time in msec,
	 * this method will check what is the total time spend in
	 * this loop and then sleep the remaining period time 
	 * (if the time taken by the loop is less than the allowed period)
	 */
	while(1) 
	{
		loop_timer.Reset();
		test.Reset();
		pthread_testcancel();     // Thread cancellation point.
		CheckConfig();            // Check for Config Requests
		CheckCommands();          // Check for Commands
		BuildMap();		          // Launch the MrICP on two laser scans
		RefreshData();            // Update Data
		//time_elapsed = loop_timer.TimeElapsed();
		//cout<<"\n Min Loop took="<<time_elapsed/1e3<<"usec";
		loop_timer.Synch(this->period*1e3);
		time_elapsed = map_timer.TimeElapsed();
		if( time_elapsed >= this->map_saving_period*1e6)
		{
		    this->map->SavePgm();
			map_timer.Reset();
		}
		//time_elapsed = loop_timer.TimeElapsed();
		//cout<<"\n Time Synch="<<time_elapsed/1e3<<"usec";	
	}
	pthread_exit(NULL);
}
void MrIcpDriver::CheckConfig()
{
	void *client;
	unsigned char buffer[PLAYER_MAX_REQREP_SIZE];
	size_t buffer_len;
  	if ((buffer_len=this->GetConfig(this->position_out_id, &client, &buffer, sizeof(buffer), NULL)) > 0)
  	{
			cout<<"\n	--> Recieved Position request\n"; 
			fflush(stdout);
		if(debug)
		{
			cout<<"\n	--> Recieved Position request\n"; 
			fflush(stdout);
		}
		switch (buffer[0])
		{
		case PLAYER_POSITION_GET_GEOM_REQ:  // Return the robot geometry.
			if(buffer_len != 1) 
			{
				PLAYER_WARN("Get robot geom config is wrong size; ignoring");
				if(this->PutReply(this->position_out_id, client, PLAYER_MSGTYPE_RESP_NACK, NULL))
					PLAYER_ERROR("failed to PutReply");
				break;
			}
			else
			{
				uint16_t reptype;
				uint8_t req;
			    struct timeval tv;
		  		// Request geom from the underlying position driver
		  		if(position_in_exists)
		  		{
				   	req = PLAYER_POSITION_GET_GEOM_REQ;
				   	if (this->position_driver->Request(this->position_in_id,(void*)this, (void*) &req, 1,&tv , &reptype,(void*) &position_in_geom, sizeof(position_in_geom),&tv) < 0)
			   		{
				    	PLAYER_ERROR("Unable to get Position geometry from the underlying Position Interface");
				    	exit(1);
			   		}
					position_out_geom.subtype = PLAYER_POSITION_GET_GEOM_REQ;
					position_out_geom.pose[0] = position_in_geom.pose[0];
					position_out_geom.pose[1] = position_in_geom.pose[1];
					position_out_geom.pose[2] = position_in_geom.pose[2];
					position_out_geom.size[0] = position_in_geom.size[0];
					position_out_geom.size[1] = position_in_geom.size[1];
		  		}
		  		else // Default Value if Position:0 not found
		  		{
					position_out_geom.subtype = PLAYER_POSITION_GET_GEOM_REQ;
					position_out_geom.pose[0] = htons((short) (-110));
					position_out_geom.pose[1] = htons((short) (0));
					position_out_geom.pose[2] = htons((short) (0));
					position_out_geom.size[0] = htons((short) (370));
					position_out_geom.size[1] = htons((short) (370));
		  		}
				if (this->PutReply(this->position_out_id, client, PLAYER_MSGTYPE_RESP_ACK, &this->position_out_geom, sizeof(this->position_out_geom), NULL)!=0)
					PLAYER_ERROR("failed to PutReply");
			}
				break;
		case PLAYER_POSITION_SET_ODOM_REQ:  // Sets the robot geometry.
				this->PutReply(this->position_out_id,client, PLAYER_MSGTYPE_RESP_NACK, NULL, 0, NULL);  
				break;
		case PLAYER_POSITION_RESET_ODOM_REQ:  // Resets the robot geometry.
                        PLAYER_WARN("Got robot reset request!! Woohoo"); 	
			if(buffer_len != sizeof(player_position_resetodom_config_t)) 
			{
				PLAYER_WARN("Get robot geom config is wrong size; ignoring");
				if(this->PutReply(this->position_out_id, client, PLAYER_MSGTYPE_RESP_NACK, NULL))
					PLAYER_ERROR("failed to PutReply");
				break;
			}
			else
				ResetMap();
                                if(this->PutReply(this->position_out_id, client,PLAYER_MSGTYPE_RESP_ACK, NULL)){
                                  PLAYER_ERROR("Failed to inform client of reset");  
                                }

			break;
		case PLAYER_POSITION_MOTOR_POWER_REQ:
				this->PutReply(this->position_out_id,client, PLAYER_MSGTYPE_RESP_NACK, NULL, 0, NULL);  
	        	break;
		default:
				PLAYER_WARN1("\nreceived unknown config type %d", buffer[0]);
	    		this->PutReply(this->position_out_id,client, PLAYER_MSGTYPE_RESP_NACK, NULL, 0, NULL);  
		}
  	}
  	if ((buffer_len=this->GetConfig(this->map_id, &client, &buffer, sizeof(buffer), NULL)) > 0)
  	{
  		switch(buffer[0])
  		{
    		case PLAYER_MAP_GET_INFO_REQ:
      			HandleGetMapInfo(client, buffer, buffer_len);
      		break;
		    case PLAYER_MAP_GET_DATA_REQ:
		      	HandleGetMapData(client, buffer, buffer_len);
		     	break;
		    default:
		      	PLAYER_ERROR("got unknown config request; ignoring");
		      	if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL) != 0)
		        	PLAYER_ERROR("PutReply() failed");
		      	break;
  		}
  	}
  	return;
}
 void MrIcpDriver::CheckCommands()
{
	if( this->GetCommand(this->position_out_id, &this->position_out_cmd, sizeof(this->position_out_cmd), NULL) > 0 )
	{
	 	if (!this->position_in_exists)
	 	{
	  		cout<<"\n	--->>>No Input Position Driver Initialized";
	  		fflush(stdout);	 		
	  		return;
	 	}
	  	this->position_driver->PutCommand(this->position_in_id, (void*) &position_out_cmd,sizeof(position_out_cmd), NULL);
	}
  	return;
}
// Handle map info request
void MrIcpDriver::HandleGetMapInfo(void *client, void *request, int len)
{
  	if(len != sizeof(map_info.subtype))
  	{
    	PLAYER_ERROR2("config request len is invalid (%d != %d)", len, sizeof(map_info.subtype));
    	if (PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL) != 0)
      		PLAYER_ERROR("PutReply() failed");
   	 	return;
  	}
  	if(this->map->occ_grid == NULL)
  	{
    	PLAYER_ERROR("NULL map data");
    	if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL) != 0)
      		PLAYER_ERROR("PutReply() failed");
    	return;
  	}

  	map_info.subtype = ((player_map_info_t*)request)->subtype;
  	map_info.scale = htonl((uint32_t)rint(1e3/this->map_resolution));
  	map_info.width =  htonl((uint32_t) (int)ceil(2*map_size/map_resolution));
  	// Here we add the Meta Data Row
  	map_info.height = htonl((uint32_t) (int)ceil(2*map_size/map_resolution + 1));

  	if (PutReply(client, PLAYER_MSGTYPE_RESP_ACK, &map_info, sizeof(map_info),NULL) != 0)
    	PLAYER_ERROR("PutReply() failed");
  	return;
}

// Handle map data request
void MrIcpDriver::HandleGetMapData(void *client, void *request, int len)
{
  	int i, j,oi, oj, si, sj,reqlen,last_row = map->size_y -1;
  	double prob;
	int16_t temp;
  	player_map_data_t data;
  	reqlen = sizeof(data) - sizeof(data.data);
  	// check if the config request is valid
  	if(len != reqlen)
  	{
    	PLAYER_ERROR2("config request len is invalid (%d != %d)", len, reqlen);
	    if(PutReply(client, PLAYER_MSGTYPE_RESP_NACK,NULL) != 0)
    	  	PLAYER_ERROR("PutReply() failed");
    	return;
  	}

  	// Construct reply
  	memcpy(&data, request, len);

  	oi = ntohl(data.col);
  	oj = ntohl(data.row);
  	si = ntohl(data.width);
  	sj = ntohl(data.height);
  	cout<<"\n	--->>> Columns="<<oi<<" Rows="<<oj<<" width="<<si<<" height="<<sj;

  	// Grab the Information from the occupancy data
  	for(j = oj; j < (sj+oj); j++)
  	{
  		// Proccess Last Row with the patch data
  		if(j == last_row )
  		{
  			// Saving Creation Map Time Stamp 
 			gettimeofday(&map_timestamp,NULL);
			// Storing Map ID can be between -127 and 127 (assumed +ve all the time)
			data.data[0 + (j-oj) * si] = map_number;
			// Storing Robot ID can be between -127 and 127 (assumed +ve all the time)			
			data.data[1 + (j-oj) * si] = robot_id;
			/* Storing Pose X */
			int8_t* offset = data.data + (j-oj)*si+2; 
			temp = (int)(global_pose.p.x*1e3);
			memcpy(offset,&temp, sizeof(temp));
			offset += sizeof(temp);   
			// Storing Pose Y 
			temp = (int)(global_pose.p.y*1e3);
			memcpy(offset,&temp, sizeof(temp));
			offset += sizeof(temp);   
			// Storing Pose Phi 
			temp = (int)(global_pose.phi*1e3);
			memcpy(offset,&temp, sizeof(temp));
			offset += sizeof(temp);   
			/* Encapsulating Map time stamp
			 * Timeval consists of two value, tv_sec and tv_usec both of
			 * long unsigned int unit32_t (4 bytes), representing time since
			 * the epoch.
			 */
			uint32_t time_temp;
			// Seconds
			time_temp = map_timestamp.tv_sec;
			memcpy(offset,&time_temp, sizeof(time_temp));
			offset += sizeof(time_temp);   
			// usec
			time_temp = map_timestamp.tv_usec;
			memcpy(offset,&time_temp, sizeof(time_temp));
			offset += sizeof(time_temp);   
			/* The Rest of the Row is ignored for now, more data will
			 * be encapsulated later on
			 */
  			continue;
  		}
    	for(i= oi; i < (si+oi); i++)
    	{
      		if(((i-oi) * (j-oj)) <= PLAYER_MAP_MAX_CELLS_PER_TILE)
      		{
        		if(MAP_VALID(this, i, j))
        		{
       				prob = map->occ_grid[i][j].prob_occ;
       				//cout<<"\n prob ="<<prob<<" found occupied"<<found_occ;
       				if(this->playerv_debug)
       				{
	        			if(prob > 0.9)
	        				data.data[(i-oi) + (j-oj) * si] = +1;
	        			else if(prob < 0.1)
	        				data.data[(i-oi) + (j-oj) * si] = -1;
	        			else
	        				data.data[(i-oi) + (j-oj) * si] =  0;
       				}
       				else
       				{
       					uint8_t value = (uint8_t)(double(255)*prob);
       					memcpy(data.data + (i-oi) + (j-oj)*si,&value, sizeof(value));
       					//if(prob <= 0.5)
       					//{
       					//	printf("\n Prob =%f",prob);
       					//	printf(" Scaled Before Casting=%f",double(255)*prob);
       					//	printf(" After Scaled Prob=%u",value);
       					//}
       				}
        		}
        		else
        		{
          			PLAYER_WARN2("requested cell (%d,%d) is offmap", i+oi, j+oj);
          			data.data[i + j * si] = 0;
	        		cout<<"\nData Sent";fflush(stdout);
        		}
      		}
     		else
      		{
        		PLAYER_WARN("requested tile is too large; truncating");
        		cout<<"\nMap Too Large";fflush(stdout);
        		if(i == 0)
        		{
          			data.width = htonl(si-1);
          			data.height = htonl(j-1);
        		}
        		else
       			{
          			data.width = htonl(i);
          			data.height = htonl(j);
        		}
      		}
    	}
  }
  	// Send map info to the client
  	//if(PutReply(client, PLAYER_MSGTYPE_RESP_ACK, &data,sizeof(data) - sizeof(data.data) + ntohl(data.width) * ntohl(data.height),NULL) != 0)
	if(PutReply(client, PLAYER_MSGTYPE_RESP_ACK, &data,ntohl(data.width) * ntohl(data.height) +(sizeof(data) - sizeof(data.data)) ,NULL) != 0)
	    	PLAYER_ERROR("PutReply() failed");
	//cout<<"\n Size Header: "<<(ntohl(data.width) * ntohl(data.height) +(sizeof(data) - sizeof(data.data)));
  	return;
}
void MrIcpDriver::RefreshData()
{
  	// Write position data//
	this->position_out_data.xpos = HTOPL(this->px * 1e3); // changing to mm because of player position interface 
	this->position_out_data.ypos = HTOPL(this->py * 1e3); // changing to mm because of player position interface 
	this->position_out_data.yaw =  HTOPL(RTOD(this->pa)); // changing to degrees because of player position interface 
	this->position_out_data.xspeed = HTOPL(this->speed * 1e3);
 	this->position_out_data.yawspeed = HTOPL(RTOD(this->turn_rate));
  	PutData(this->position_out_id,(void*)&(this->position_out_data),sizeof(player_position_data_t), &this->laser_timestamp);
	return;
};
void MrIcpDriver::SetupPositionDriver()
{
	Pose initial_pose;
	// Subscribe to the odometry driver
	this->position_driver = deviceTable->GetDriver(this->position_in_id);
	if (!this->position_driver)
	{
		PLAYER_ERROR("Couldn't Connect to position driver");
		return;
	}
	if (this->position_driver->Subscribe(this->position_in_id) != 0)
	{
		PLAYER_ERROR("Couldn't subscribe to position device");
		return;
	}
	initial_pose = GetOdomReading();
	this->px = initial_pose.p.x;
	this->py = initial_pose.p.y;
	this->pa = initial_pose.phi;
}
void MrIcpDriver::SetupLaser(int index)
{
	uint8_t req;
	uint16_t reptype;
    player_laser_geom_t geom;
    struct timeval tv;
	this->laser_driver[index] = deviceTable->GetDriver(this->laser_id[index]);
	if (!this->laser_driver)
	{
		PLAYER_ERROR("Unable to connect to laser Driver");
		return;
	}
	if (this->laser_driver[index]->Subscribe(this->laser_id[index]) != 0)
	{
	    PLAYER_ERROR("Unable to subscribe to laser device");
	    return;
	}
	// Get the laser geometry
   	req = PLAYER_LASER_GET_GEOM;
   	if (this->laser_driver[index]->Request(this->laser_id[index],(void*)this, (void*) &req, 1,&tv , &reptype,(void*) &geom, sizeof(geom),&tv) < 0)
   	{
	    PLAYER_ERROR("Unable to get laser geometry");
	    return;
   	}
  	// Get the laser pose relative to the robot center of Rotation
  	laser_pose[index].p.x = ((int16_t) ntohs(geom.pose[0])) / 1000.0;
  	laser_pose[index].p.y = ((int16_t) ntohs(geom.pose[1])) / 1000.0;
  	laser_pose[index].phi = ((int16_t) ntohs(geom.pose[2])) * M_PI / 180.0;
  	//if (this->debug)
	  	cout<<"\n Laser["<<index<<"] Pose --> X="<<laser_pose[index].p.x<<" Y="<<laser_pose[index].p.y<<" Theta="<<laser_pose[index].phi;
};
Pose MrIcpDriver::GetOdomReading()
{
	Pose P;
	size_t size;
	player_position_data_t data;
	// Get the odom device data.
	size = this->position_driver->GetData(this->position_in_id,(void*) &data,sizeof(data), &this->position_timestamp);
	if (size == 0)
		return P;
	// Get the pose
	P.p.x = (double) ((int32_t) ntohl(data.xpos)) / 1000.0;
	P.p.y = (double) ((int32_t) ntohl(data.ypos)) / 1000.0;
	P.phi = (double) ((int32_t) ntohl(data.yaw)) * M_PI / 180;
	if(this->debug)
		cout<<"\n	--->>> Odom pose from Position:0 XYTheta=["<<P.p.x<<"]["<<P.p.y<<"]["<<P.phi<<"]";
	return P;
};
// Check if the laser beam is in the allowed range
int MrIcpDriver::InRange(double angle,int laser_index)
{
	for(int i=0;i<range_count[laser_index];i++)
	{
//		if(RTOD(angle)>60)
//			cout<<"\n Laser:"<<laser_index<<" Angle="<<RTOD(angle)<<" Min"<<range[laser_index][i].begins<<" MAX"<<range[laser_index][i].ends;
		if(range[laser_index][i].begins <= RTOD(angle) && RTOD(angle) <= range[laser_index][i].ends)
			return 0; // Allowed Beam
	}
	return -1; // Beam to be ignored
};
// Read Laser Data from all the drivers
vector <Point>  MrIcpDriver::GetLaserSample()
{
	size_t size;
	player_laser_data_t data;
  	struct timeval currtime;
  	double t1,t2, min_angle, scan_res, range_res,r,b,time_diff;
  	vector<Point> laser_set;
	Point p;
	gettimeofday(&currtime,NULL);
	//Clear the previous Laser set used for the occupance grid generation
	occ_laser_set.clear();
  	// Get the laser device data.
  	for(int index=0;index<number_of_lasers;index++)
  	{
	  	size = this->laser_driver[index]->GetData(this->laser_id[index], (void*) &data,sizeof(data), &this->laser_timestamp);
	  	if (size == 0)
	  	{
	  		cout<<"\n	--->>>Error Reading From Laser";
	  		fflush(stdout);
		    return laser_set;
	  	}
		t1 = ((double) currtime.tv_sec  + (double) currtime.tv_usec/1e6);
		t2 = ((double) last_time[index].tv_sec + (double) last_time[index].tv_usec/1e6);
	    time_diff = t1 -t2;
	  	if (time_diff < 0.1)
	  	{
	  		if(this->debug)
	  			cout<<"\n	--->>> Time Since Last Read is= "<<(t1-t2)<<" msec";
	    	continue;
	  	}
	  	this->last_time[index] = currtime;
	  	min_angle = DTOR((int16_t)ntohs(data.min_angle)/1e2);
	  	scan_res  = DTOR((int16_t)ntohs(data.resolution)/1e2);
	  	range_res = (int16_t) ntohs(data.range_res); 
	  	this->scan_count = ntohs(data.range_count);
	    if(this->debug)
	    	cout<<"\n Scan Count="<<this->scan_count;
	  	for(int i=0;i < scan_count ;i++)
	  	{
	    	// convert to mm, then m, according to given range resolution
		    r = ntohs(data.ranges[i]) * range_res / 1e3;
	    	b = min_angle + (i * scan_res); 		    // compute bearing
	    	//cout<<"\n Bearing:"<<RTOD(b);
	    	if(InRange(b,index)!=0)
	    		continue;
		    // Transfer from Polar to Cartesian coordinates
	    	p.x = r * cos(b);
	    	p.y = r * sin(b);
	    	p.laser_index = index;
	    	// Transfer all the laser Reading into the Robot Coordinate System
	    	p = TransformToGlobal(p,laser_pose[index]);
			// Filter max ranges for alignment and use all for Occ-grid
	    	if (this->use_max_range!=0)
	    	{
		    	if (r >= this->minr && r <= this->maxr && (i%sparse_scans_rate)==0)
	  				laser_set.push_back(p);
	  			occ_laser_set.push_back(p);
	  		}
	  		else
	  		{
	    		// Use only informative data for the scan allignement and Occ-grid
		    	if (r >= this->minr && r <= this->maxr)
		    	{
		    		// Get samples according to sparse resolution
		    		if ((i%sparse_scans_rate)==0)
	  					laser_set.push_back(p);
	  				// Use All the samples for OG-Map
	  				occ_laser_set.push_back(p);
		    	}
	  		}
	  	}
  	}
 return laser_set;//success
}
Point MrIcpDriver::TransformToGlobal(Point p,Pose pose)
{
	// Rotate + Translate
	Point temp = p;
	p.x = temp.x*cos(pose.phi) - temp.y*sin(pose.phi) + pose.p.x ;
	p.y = temp.x*sin(pose.phi) + temp.y*cos(pose.phi) + pose.p.y ;
	return p;
};
Pose MrIcpDriver::TransformToGlobal(Pose p,Pose pose)
{
	// Rotate + Translate
	Point temp = p.p;
	p.p.x = temp.x*cos(pose.phi) - temp.y*sin(pose.phi) + pose.p.x ;
	p.p.y = temp.x*sin(pose.phi) + temp.y*cos(pose.phi) + pose.p.y ;
	p.phi = NORMALIZE(p.phi + pose.phi);
	return p;
};
// transfers from Pixel to the Map coordinate
Point MrIcpDriver :: ConvertPixel(Point  p) 
{
	p.x = ( p.x*this->map_resolution - this->map_size) ;
	p.y = (-p.y*this->map_resolution + this->map_size) ;
	return p;
};
// transfers from Map into the Pixel Coordinate 
Point MrIcpDriver :: ConvertToPixel(Point p) 
{
	//  This is a NxN Map with N = 2*map_size
	p.x = rint (( p.x + this->map_size)/this->map_resolution);
	p.y = rint ((-p.y + this->map_size)/this->map_resolution);
	return p;
};
mapgrid_t MrIcpDriver::ComputeRangeProb(double range,bool free)
{
	double bad_range;
	mapgrid_t prob;
	// 2 , 3 or 5 cm error based on the laser specifications
	// for the hokouo this should be modified to a constant of 2 cm
	if (range <=2)
		bad_range = 0.02;
	else if (range>2 && range<=4)
		bad_range = 0.03;
	else if (range>4)
		bad_range = 0.05;
	if (free)
	{
		prob.prob_free = range /(range + bad_range); 
		prob.prob_occ  =  1 - prob.prob_free;
	}
	else
	{
		prob.prob_occ  = range /(range + bad_range);
		prob.prob_free = 1 - prob.prob_occ; 
	}
	if(range == 0)
	{
		prob.prob_free = 1;
		prob.prob_occ  = 0;
	}
	//cout<<"\n Prob Free="<<prob.prob_free<<" Prob Occ="<<prob.prob_occ;
	return prob;	
}
void MrIcpDriver::AddToMap(vector<Point> laser_data,Pose pose)
{
	Point p,pixel_point,d,in_p;
	Pose relative_laser_pose;
	double dist,color_prob,gradient;
	double x_free,x_occ,x_unknown,range,normalizer;
  	mapgrid_t sensor_prob;
	int steps;
	for(unsigned int i=0;i<laser_data.size();i++)
	{
		p = TransformToGlobal(laser_data[i],pose);
		pixel_point = ConvertToPixel(p);
		if(pixel_point.x > (map_size*2.0/map_resolution)-1 || pixel_point.y >(2.0*map_size/map_resolution) - 2 // -2 because the last row will hold the meta data
		|| pixel_point.x < 0 || pixel_point.y < 0)
		{
			//cout<<"\n	--->>> Map Size Limitations Exceeded Creating New Patch <<<---";
			cout<<"\n	--->>> Map Size Limitations , New Data Ignored <<<---";
			//ResetMap();
			fflush(stdout);
			return;
		}
		// Round the pose to the closest resolution
//		p.x = map_resolution*rint(p.x/map_resolution);
//		p.y = map_resolution*rint(p.y/map_resolution);
		// line parameters
		relative_laser_pose = TransformToGlobal(laser_pose[laser_data[i].laser_index],pose);
		d.x = p.x - relative_laser_pose.p.x; // Ray distance X in meters
		d.y = p.y - relative_laser_pose.p.y; // Ray distance Y in meters
		// Ray Total length meters
		dist = sqrt(d.x * d.x + d.y * d.y);
		steps = (int) floor(dist / map_resolution) + 1;
		d.x /= (steps);
		d.y /= (steps);
		// Traverse the line segment and update the grid
		Point temp;
		// The free Cells
		for (int j = 0; j < steps; j++)
		{
		  in_p.x = relative_laser_pose.p.x + d.x * j;
		  in_p.y = relative_laser_pose.p.y + d.y * j;
		  temp = ConvertToPixel(in_p);
		  // Prior Probability Knowledge
		  x_free = this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_free;
		  x_occ  = this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_occ;
		  x_unknown = 1 - (x_free + x_occ);
		  range = sqrt(pow(in_p.x - relative_laser_pose.p.x,2)+pow(in_p.y - relative_laser_pose.p.y,2));
		  // Range Sensor Probability
		  // Bayesian Update
		  if (dist > this->maxr)
		  {
		  	if(range > this->use_max_range)
		  		break;
		  	sensor_prob.prob_free = free_space_prob;
		  	sensor_prob.prob_occ  = (1 - free_space_prob)/2.0;
		  }
		  else
		  {
		 	sensor_prob = ComputeRangeProb(range,1);		
		  }
 	  	  normalizer  = x_free*sensor_prob.prob_free + x_occ*sensor_prob.prob_occ +
		  			    x_unknown*(1-(sensor_prob.prob_free + sensor_prob.prob_occ ));
		  this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_free = x_free*sensor_prob.prob_free/normalizer;
		  this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_occ  = x_occ *sensor_prob.prob_occ/normalizer;	  		  
		  //cout<<"\n Prob OCC ="<<x_occ *sensor_prob.prob_occ/normalizer<<" Prob Free="<<x_free*sensor_prob.prob_free/normalizer;
		  // Draw the Probability Gradient into the buffer
		  color_prob =  this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_occ;
		  if(color_prob>1 || color_prob<0)
		  {
		  	cout <<"\n UNEXPECTED Probability !!! "<< color_prob;
		  }
		  gradient = 255.0 - color_prob * 255.0;
		  map->DrawPixel((int)(gradient),(int)(gradient),(int)(gradient),(int)temp.x,(int)temp.y);  
		}
		// The end point is occupied
		if (dist < this->maxr) 
		{
		  	in_p.x = p.x;
		  	in_p.y = p.y;
		  	temp = in_p;
		  	temp = ConvertToPixel(in_p);
		  	// Prior Probability
		  	x_free = this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_free;
		  	x_occ  = this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_occ;
		  	x_unknown = 1 - (x_free + x_occ);
		  	range = sqrt(pow(in_p.x - relative_laser_pose.p.x,2)+pow(in_p.y - relative_laser_pose.p.y,2));
		  	// Range Sensor Probability
		  	sensor_prob = ComputeRangeProb(range,0);
		  	normalizer = x_free*sensor_prob.prob_free + x_occ*sensor_prob.prob_occ +
		  			   x_unknown*(1-(sensor_prob.prob_free + sensor_prob.prob_occ ));
		  	// Bayesian Update
		  	this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_free = x_free*sensor_prob.prob_free/normalizer;
		  	this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_occ  = x_occ *sensor_prob.prob_occ/normalizer;	  		  
			//cout<<"\n Prob OCC ="<<x_occ *sensor_prob.prob_occ/normalizer<<" Prob Free="<<x_free*sensor_prob.prob_free/normalizer;
		  	// Draw the Probability Gradient into the buffer
			color_prob =  this->map->occ_grid[(int)(temp.x)][(int)(temp.y)].prob_occ;
			gradient = 255.0 - color_prob * 255.0;
			//cout<<" Gradient= "<<gradient;
			map->DrawPixel((int)(gradient),(int)(gradient),(int)(gradient),(int)temp.x,(int)temp.y);  
			// Add the Point to the global set
			if (! this->map->occ_grid[(int)(pixel_point.x)][(int)(pixel_point.y)].added)	
			{
				this->map->occ_grid[(int)(pixel_point.x)][(int)(pixel_point.y)].added = true;
				this->map_points.push_back(p);
			}
		}
	}
};
// Only get the Existing Map points that are useful for Allignement
void MrIcpDriver::GenerateLocalMap(Pose pse)
{
	double farest_laser_dist=0,dist; //num_pixels;
	//Point location,grid_start,temp;
	//location.x = pse.p.x;
	//location.y = pse.p.y;
	local_map.clear();
	for(int i=0;i<this->number_of_lasers;i++)
	{
		// Get the distance from the Robot's Origin to the Laser Position
		dist = sqrt(pow(laser_pose[i].p.x,2)+pow(laser_pose[i].p.y,2)); 
		if( dist > farest_laser_dist )
			farest_laser_dist = dist;
	}
	for (unsigned int i=0;i<map_points.size();i++)
	{
		if (sqrt (pow(pse.p.x - map_points[i].x,2) + pow(pse.p.y - map_points[i].y,2)) <= (maxr + farest_laser_dist + 0.2))
			local_map.push_back(map_points[i]);
	}
//	num_pixels = (farest_laser_dist + this->maxr) /this->map_resolution + 10;
//	location = ConvertToPixel(location);
//	grid_start.x = location.x - num_pixels; 
//	if(grid_start.x < 0) grid_start.x = 0;
//	grid_start.y = location.y - num_pixels; 
//	if(grid_start.y < 0) grid_start.y = 0;
//    cout<<"\nStart grid: "<<grid_start.x<<" y:"<<grid_start.y<<" pixels:"<<num_pixels; fflush(stdout);
//	for(int i= (int)(grid_start.x) ; i< (2*num_pixels + grid_start.x); i++)
//		for(int j=(int)(grid_start.y);j<(2*num_pixels + grid_start.y); j++)
//		{
//			 y is -2 because last row is meta data
//			if(i<(map->size_x - 1)  && j<(map->size_y - 2)) 
//				if (map->occ_grid[i][j].prob_occ > 0.9)
//				{
//					temp.x = i;
//					temp.y = j;
//					local_map.push_back(ConvertPixel(temp));
//				}
//		}
};
void MrIcpDriver::BuildMap()     
{
//	double estimated_delta_d,estimated_delta_phi;
	this->delta_time = delta_t_estimation.TimeElapsed()/1e6;
	if(this->reset_timer)
		delta_t_estimation.Reset();
	if (!sample_initialized)
	{
		laser_set_1 = GetLaserSample();
		if (laser_set_1.size() != 0)
			sample_initialized = TRUE;
		else
			return;
		// Read Pose if postion driver exists
		if(this->position_driver)	pose_1 = GetOdomReading(); 
		global_pose.p.x = global_pose.p.y = global_pose.phi = 0;
		AddToMap(laser_set_1,global_pose);
		gettimeofday(&last_delta,NULL);
		return;
	}
	laser_set_2 =  GetLaserSample();
	if (laser_set_2.size() == 0 || laser_set_1.size() == 0)
		return;

	//cout<<"\n Laser Set 1 Size:"<<laser_set_1.size()<<" Laser Set 2 Size:"<<laser_set_2.size();
	// Read Pose if position driver exists
	if(this->use_odom)
	{
		pose_2 = GetOdomReading(); 
		delta_pose.phi = NORMALIZE(pose_2.phi - pose_1.phi);
		delta_pose.p.x =  (pose_2.p.x - pose_1.p.x)*cos(pose_1.phi) + (pose_2.p.y - pose_1.p.y)*sin(pose_1.phi) ;
		delta_pose.p.y = -(pose_2.p.x - pose_1.p.x)*sin(pose_1.phi) + (pose_2.p.y - pose_1.p.y)*cos(pose_1.phi) ;
	}
	else
	{
		delta_pose.phi = 0; delta_pose.p.x = 0; delta_pose.p.y = 0;
	}
	if (this->debug)
	{
		cout<<"\n POSE 1 XYQ["<<pose_1.p.x<<"]["<<pose_1.p.y<<"]["<<pose_1.phi<<"]  ";
		cout<<" POSE 2 XYQ["<<pose_2.p.x<<"]["<<pose_2.p.y<<"]["<<pose_2.phi<<"]";
	}
	// Estimate the movement based on the last linear and angular velocity
	//estimated_delta_d  = this->delta_time * this->speed;
	//estimated_delta_phi= this->delta_time * this->turn_rate;
	//cout<<"\n Delta t:"<<delta_time<<" Estimeted d:"<<estimated_delta_d<<" Estimated Phi:"<<estimated_delta_phi;
	
	// Check what is the displacement estimated by ICP
	delta_pose = icp.align(laser_set_1,laser_set_2,delta_pose, gate1, nit, interpolate);
	if(delta_pose.p.x ==-1 && delta_pose.p.y ==-1 && delta_pose.phi==-1)
	{
		cout <<"\nWARNING: possible misalignment ICP: 1 - skipping scan";
		//laser_set_1 = laser_set_2;
		return;
	}
	global_pose = TransformToGlobal(delta_pose,global_pose);
	// Is the ICP estimation more than what we excpect ?? If yes then ignore this set
	/*if(sqrt(pow(delta_pose.p.x,2)+pow(delta_pose.p.y,2)) > (estimated_delta_d +0.1 ) || abs(delta_pose.phi) > abs(estimated_delta_phi + DTOR(5)))
	{
		cout<<"\n Delta Pose:1 x:"<<delta_pose.p.x <<" Y="<<delta_pose.p.y<<" Phi"<<delta_pose.phi;
		cout <<"\nWARNING: possible misalignment - skipping scan";
		this->reset_timer = false;
		return;
	}*/
	this->reset_timer = true;
	if (!this->map_points.size())
	{
		sample_initialized = false;
		return;
	}
	this->speed = sqrt(pow(delta_pose.p.x,2) + pow(delta_pose.p.y,2))/this->delta_time;
	this->turn_rate = delta_pose.phi/this->delta_time;	

	GenerateLocalMap(global_pose);
	global_pose = icp.align(this->local_map,laser_set_2,global_pose, gate1, nit, interpolate);
	if(global_pose.p.x ==-1 && global_pose.p.y ==-1 && global_pose.phi==-1)
	{
		cout <<"\nWARNING: possible misalignment ICP: 2 - skipping scan";
		global_pose.p.x = global_pose_prev.p.x;
		global_pose.p.y = global_pose_prev.p.y;
		global_pose.phi = global_pose_prev.phi;
		//laser_set_1 = laser_set_2;
		return;
	}
	//cout<<"\n Delta Pose:2 x:"<<global_pose.p.x<<" y:"<<global_pose.p.x<<" phi:"<<global_pose.phi;
	global_pose = icp.align(this->local_map,laser_set_2,global_pose, gate2, nit, interpolate);
	if(global_pose.p.x ==-1 && global_pose.p.y ==-1 && global_pose.phi==-1)
	{
		cout <<"\nWARNING: possible misalignment ICP: 3 - skipping scan";
		global_pose.p.x = global_pose_prev.p.x;
		global_pose.p.y = global_pose_prev.p.y;
		global_pose.phi = global_pose_prev.phi;
		//laser_set_1 = laser_set_2;
		return;
	}
	//cout<<"\n Delta Pose:3 x:"<<global_pose.p.x<<" y:"<<global_pose.p.x<<" phi:"<<global_pose.phi;

	// Serve Data to Position Interface
	this->global_pose_prev.p.x = this->px = global_pose.p.x;
	this->global_pose_prev.p.y = this->py = global_pose.p.y;
	this->global_pose_prev.phi = this->pa = NORMALIZE(global_pose.phi);
	// Use ALL the Laser data for the occupancy grid update
	AddToMap(occ_laser_set,global_pose);

	// Perform the ICP on Next Laser Scan
	laser_set_1 = laser_set_2;
	if(this->position_driver)	pose_1 = pose_2;
};

void MrIcpDriver::ConnectPatches()
{
	int patch_number=0;
	gchar * patch;
	patch = g_strdup_printf("%sMAP_PATCH%d.png",map_path,patch_number++);
	while(is_file(patch))
	{
		cout<<patch<<endl; fflush(stdout);
		patch = g_strdup_printf("%sMAP_PATCH%d.png",map_path,patch_number++);
	}
}
 



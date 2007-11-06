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
 * SSPP_driver.cpp, v1.0 1/11/2007 
 * This is a Path Planning driver for online path planning using an offline generated Search Space.
 * It's a very efficient method for path planning in a mapped environment with tight spaces. The path
 * Planning takes into consideration the size of the robot and generates a search space of the environment
 * that can be used for online Path Planning.
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
#include "pathplanner.h"

using namespace std;
using namespace CasPlanner;
/** @ingroup drivers */
/** @{ */
/** @defgroup driver_SSPP SSPP
 * @brief Search Space Path Planner
 * 

This is a Path Planning Driver suitable for path planning online in a well known environments
where a map is provided. It's based on an efficient sampling method and collision detection that
makes path planning for tight and narrow environments possible and fast. Check my website for
more information http://www.tarektaha.com

This driver is created to support UTS CAS (Centre of Autonomous Systems) mobile Platforms.
I would appreciate any feedback and recommendations that can lead to improving the performance
of this driver.

@par Compile-time dependencies

- PathPlanner Library

@par Provides

The SSPP driver provides the following device interfaces:

- "planner" @ref player_interface_planner
  - This interface returns odometry data.

@par Supported configuration requests

- "Position" @ref player_interface_planner:
  - PLAYER_PLANNER_MAX_WAYPOINTS 
  - PLAYER_PLANNER_DATA_STATE
  - PLAYER_PLANNER_CMD_GOAL
  - PLAYER_PLANNER_REQ_GET_WAYPOINTS
  - PLAYER_PLANNER_REQ_ENABLE 

@par Configuration file options

- traversableDist (float) 
	- Default: 1.0
	-
- narrowestPassage (float)
	- Default: 0.987
	- 
- distanceToGoal (float) 
	- Default: 2
	-
- bridgeLen (float)
	- Default: 2
	-
- bridgeRes (float) 
	-Default: 0.1
	-
- regGridLen (float) 
	-Default: 0.2
	-
- regGridConRad (float)
	- Default: 0.4
	-
- obstPenalty (float)
	- Default: 3.0
	-
- bridgeConRad (float) 
	- Default: 0.5
	-
@par Example 

@verbatim
driver
(
  name "SSPP"
  plugin "SSPP.so"
  provides ["planner:0"]
  requires ["output:::position2d:1" "input:::position2d:2" "map:0"] 
  traversableDist 1
  narrowestPassage 0.9
  distanceToGoal 0.2
  bridgeLen 3.0
  bridgeRes 0.2
  regGridLen 0.4
  regGridConRad 0.4
  bridgeConRad 0.5
  obstPenalty 2
)
@endverbatim

@par Authors

Tarek Taha - Centre of Autonomous Systems - University of Technology Sydney
*/
/** @} */
  /////////////////////////////////////////////////////////////
 ///                   SSPP DRIVER  Class                  ///
/////////////////////////////////////////////////////////////
class SSPP : public Driver
 {
	// Must implement the following methods.
  	public :	
	    virtual int Setup();
	    virtual int Shutdown();
	    virtual int ProcessMessage(MessageQueue * resp_queue, player_msghdr * hdr, void * data);
	    virtual	~SSPP();
	// Constructor
	public:  	SSPP(ConfigFile* cf, int section); 
	// Main function for device thread.
    private:	
    		virtual void Main();
			int  handleConfigs(MessageQueue* resp_queue,player_msghdr * hdr,void * data);
			int  handleCommands(MessageQueue* resp_queue,player_msghdr * hdr,void * data);
			int  handleData(MessageQueue * resp_queue, player_msghdr * hdr, void * data);
  			int  setupMapDevice();
  			int  setupLocalizerDevice();
			int  setupPositionDevice();  			
  			void sendPositionCommand(double x, double y, double a, unsigned char type);
  			int  setupPlanner();				
  			int  getMapInfo();
  			int  getMap();
  			void findPath();
 		 	void refreshData();     //refreshs and sends data    
			bool getWayPoint(Pose &goal);
			bool goalReached();
			void go2WayPoint();
			void stopDriving();
			Node * closestPathNode(Point location);
			Node * closestPathSeg(Point location);
			int ProcessMapInfoReq(MessageQueue* resp_queue,player_msghdr * hdr,void * data);
			int ProcessMapDataReq(MessageQueue* resp_queue,player_msghdr * hdr,void * data);
	// Position interface / IN
  	private:
  			player_devaddr_t          localizer_addr;
  		 	player_position2d_data_t  localizer_data;
			player_position2d_geom_t  geom;
			Device  			     *localizer_device,*position_device;
	// Position interface / OUT
  	private:
  			player_devaddr_t         position_out_addr;
  		 	player_position2d_data_t position_out_data;
	// Map interface
  	private:
  			player_devaddr_t 	   map_addr;
  		 	player_map_data_t 	   map_data;
			player_map_info_t      map_info;
			Device                 *map_device;   // Used to communicate with Map Driver
	// Variables
	public :
			FILE *file,*config_file;
			int mapW,mapH,numPathWayPoints;
			Node *path,*tempPath;
		    bool debug,haveGoal,recievedNewGoal,gotMap,gotNewMap,enableDrive;
    		Pose startPose,endPose,currentPose,wayPoint;
    		double robotL,robotW,narrowestPassage,mapRes;
    		double traversableDist,distanceToGoal,bridgeLen,bridgeRes,regGridLen,regGridConRad,obstPenalty,bridgeConRad;
		    Point robotCenter,mapCenter;
    		Robot *robot;
    		Map   *map;
			PathPlanner * pathPlanner;
			char * map_path;
			struct timeval last_time,current_time,laser_timestamp,position_timestamp;
};
Driver* SSPP_Init(ConfigFile* cf, int section) // Create and return a new instance of this driver
{
  	return ((Driver*) (new SSPP(cf, section)));
}

void SSPP_Register(DriverTable* table)
{
  	table->AddDriver("SSPP", SSPP_Init);
}
/* need the extern to avoid C++ name-mangling  */
extern "C"
{
  int player_driver_init(DriverTable* table)
  {
    puts("	--->>>Initializing Pluggin Driver ==>  SSPP Driver ...");
    SSPP_Register(table);
    return(0);
  }
}
		    
SSPP::SSPP(ConfigFile* cf, int section) : Driver(cf, section, true,PLAYER_MSGQUEUE_DEFAULT_MAXLEN, PLAYER_PLANNER_CODE),
mapW(0),
mapH(0),
numPathWayPoints(0),
path(NULL),
debug(false),
haveGoal(false),
recievedNewGoal(false),
gotMap(false),
gotNewMap(false),
enableDrive(true),
robot(NULL),
map(NULL),
pathPlanner(NULL)
{
	this->debug =					cf->ReadBool(section,"debug",0);
	this->traversableDist=         	cf->ReadFloat(section,"traversableDist",1.0);
    this->narrowestPassage=         cf->ReadFloat(section,"narrowestPassage",0.987);
    this->distanceToGoal=	        cf->ReadFloat(section,"distanceToGoal",0.2);
    this->bridgeLen=         		cf->ReadFloat(section,"bridgeLen",2);
    this->bridgeRes=         		cf->ReadFloat(section,"bridgeRes",0.1);
    this->regGridLen=       	  	cf->ReadFloat(section,"regGridLen",0.2);
    this->regGridConRad=         	cf->ReadFloat(section,"regGridConRad",0.4);
    this->obstPenalty=         		cf->ReadFloat(section,"obstPenalty",3.0);
    this->bridgeConRad=         	cf->ReadFloat(section,"bridgeConRad",0.5);
	
	// Must have a position device to control
  	if (cf->ReadDeviceAddr(&this->position_out_addr, section, "requires",PLAYER_POSITION2D_CODE, -1, "output") != 0)
  	{
  		puts("No Output Position interface");
    	this->SetError(-1);    
    	return;
  	}
  	else
  	{
  		if(setupPositionDevice()<0)
  		{
  			this->SetError(-1);
  			return;
  		}
  	}

  	if (cf->ReadDeviceAddr(&this->localizer_addr, section, "requires",PLAYER_POSITION2D_CODE, -1, "input") != 0)
	{
		puts("No Input Position interface");
    	this->SetError(-1);    
    	return;
  	}
	else
	{
  		if(setupLocalizerDevice() < 0)
  		{
  			this->SetError(-1);
  			return;
  		}
	}

    if(cf->ReadDeviceAddr(&(this->map_addr), section, "requires", PLAYER_MAP_CODE, -1, NULL) != 0)
    {
    	this->SetError(-1);    
    	return;
  	}
  	else
  	{
  		if(setupMapDevice() < 0)
  		{
  			this->SetError(-1);
  			return;
  		}
  	}
  	
	if(setupPlanner() < 0)
  	{
  		puts("Error Happened while initializing Planner!!!");
  		this->SetError(-1);
    	return;
  	}
  	return;
}

SSPP::~SSPP()
{
    delete robot;
    delete map;
    delete pathPlanner;		
}

int SSPP::Setup()
{
  	this->StartThread();
	return(0);
};

int SSPP::Shutdown()
{
	this->haveGoal = false;
  	this->StopThread();
	cout<<"\n Thread Killed ->"; fflush(stdout);
	cout<<" ... ShutDown FINISED\n"; fflush(stdout);
	return(0);
}; 

int SSPP::setupMapDevice()
{
	if(!(this->map_device = deviceTable->GetDevice(this->map_addr)))
  	{
    	PLAYER_ERROR("unable to locate suitable map device");
    	return -1;
  	}
  	if(map_device->Subscribe(this->InQueue) != 0)
  	{
    	PLAYER_ERROR("unable to subscribe to map device");
    	return -1;
  	}
	printf("Loading map from Map interface...\n");
  	fflush(NULL);
	if(this->getMapInfo() < 0)
    	return -1;
	if(this->getMap() < 0)
    	return -1;
	puts("Done.");
	return 0;	
}

int SSPP::getMapInfo()
{
	Message* msg;
	if(!(msg = this->map_device->Request(this->InQueue,PLAYER_MSGTYPE_REQ,PLAYER_MAP_REQ_GET_INFO,NULL, 0, NULL,false)))
	{
    	PLAYER_WARN("failed to get map info");
    	return -1;
  	}
	player_map_info_t* info = (player_map_info_t*)msg->GetPayload();
	if (pathPlanner)
		delete pathPlanner;
	if (map)
		delete map;
  	this->mapRes = info->scale;
  	this->mapW = info->width;
  	this->mapH = info->height;
  	this->mapCenter.setX(info->origin.px);
  	this->mapCenter.setY(info->origin.py);
	map  = new Map(this->mapW,this->mapH,this->mapRes,false);
	delete msg;
  	return(0);
};

int SSPP::getMap()
{
	player_map_data_t* data_req;
  	size_t reqlen;
  	int i,j,oi,oj,sx,sy,si,sj;
  	
  	reqlen = sizeof(player_map_data_t) - PLAYER_MAP_MAX_TILE_SIZE;
  	data_req = (player_map_data_t*)calloc(1, reqlen);
  	assert(data_req);
  	// Tile size
  	sy = sx = (int)sqrt(PLAYER_MAP_MAX_TILE_SIZE);
  	assert(sx * sy < (int)PLAYER_MAP_MAX_TILE_SIZE);
  	oi=oj=0;
  	while((oi < this->mapW) && (oj < this->mapH))
  	{
    	si = MIN(sx, this->mapW - oi);
    	sj = MIN(sy, this->mapH - oj);
    	data_req->col = oi;
    	data_req->row = oj;
    	data_req->width = si;
    	data_req->height = sj;
    	Message* msg;
    	if(!(msg = this->map_device->Request(this->InQueue,PLAYER_MSGTYPE_REQ,PLAYER_MAP_REQ_GET_DATA,(void*)data_req,reqlen,NULL,false)))
    	{
      		PLAYER_ERROR("failed to get map data");
      		free(data_req);
      		return(-1);
    	}
    	player_map_data_t* mapdata = (player_map_data_t*)msg->GetPayload();
    	cout<<"\n row"<<mapdata->row<<" col"<<mapdata->col<<" W:"<<mapdata->width<<" H:"<<mapdata->height;
    	// Fill the map grid with the values from the interface
    	for(j=0;j<sj;j++)
    	{
      		for(i=0;i<si;i++)
      		{
        		if(mapdata->data[j*si + i] >= 0)
        		{
          			map->grid[oi+i][sj-(oj+j+1)] = 1;
        		}
        		else
        		{
        			map->grid[oi+i][sj-(oj+j+1)] = 0;
        		}
      		}
    	}

    	delete msg;

    	oi += si;
    	if(oi >= this->mapW)
    	{
      		oi = 0;
      		oj += sj;
    	}
  	}
  	free(data_req);
  	gotMap = true;
  	return 0 ;
};

int SSPP::setupLocalizerDevice()
{
	if(!(this->localizer_device = deviceTable->GetDevice(this->localizer_addr)))
  	{
    	PLAYER_ERROR("unable to locate suitable localizing device");
    	return -1;
  	}
  	if(this->localizer_device->Subscribe(this->InQueue) != 0)
  	{
		PLAYER_ERROR("unable to subscribe to the localizer device");
    	return -1;
  	}
	return 0;	
}

int SSPP::setupPositionDevice()
{
	Pose initial_pose;
	// Subscribe to the underlyin odometry device
	if(!(this->position_device = deviceTable->GetDevice(this->position_out_addr)))
	{
		PLAYER_ERROR("unable to locate suitable position device");
	    return -1;
	}
	if(this->position_device->Subscribe(this->InQueue) != 0)
	{
		PLAYER_ERROR("unable to subscribe to position device");
	    return -1;
	}
 	// Get the robot's geometry
  	Message* msg;
  	if(!(msg = this->position_device->Request(this->InQueue,PLAYER_MSGTYPE_REQ,PLAYER_POSITION2D_REQ_GET_GEOM,NULL, 0, NULL,false)) 
  	  ||(msg->GetHeader()->size != sizeof(player_position2d_geom_t)))
  	{
    	PLAYER_ERROR("failed to get geometry of underlying position device");
    	if(msg)
      		delete msg;
    	return -1;
  	}
  	memcpy(&geom,(player_position2d_geom_t *)msg->GetPayload(),sizeof(geom));
  	robotL = geom.size.sl;
  	robotW = geom.size.sw;
  	std::cout<<"\nLength:"<<robotL<<" Width:"<<robotW;
 	std::cout<<"\n x"<<geom.pose.px<<" y:"<<geom.pose.py<<" Z:\n"<<geom.pose.pa;
 	robotCenter.setX(geom.pose.px);
 	robotCenter.setY(geom.pose.py);
 	return 0;
};

int SSPP::setupPlanner()
{
    robot= new Robot(string("Robot"),robotL,robotW,narrowestPassage,robotCenter);
    pathPlanner = new PathPlanner(robot,this->map,distanceToGoal,bridgeLen,bridgeRes,regGridLen,regGridConRad,obstPenalty,bridgeConRad);
    pathPlanner->enableBridgeTest(true);
    pathPlanner->enableRegGrid(true);
    pathPlanner->enableObstPen(true);
    pathPlanner->enableExpObst(true);
    /*
     *  Search Space generation, Please note that you will need
     * to generate only one search space for each map. After
     * the Search Space generation you can search for as many paths
     * as you want.
     */
    pathPlanner->generateSpace();
    if(this->debug)
    	pathPlanner->drawSearchSpace();
    pathPlanner->printNodeList();    
    cout<<"\nPlanner has been Initiated Properly\n"; fflush(stdout);
    return 0;	
};

bool SSPP::goalReached()
{
	if(Dist(currentPose.p,endPose.p) < distanceToGoal)
		return true;
	else
		return false;
};

void SSPP::stopDriving()
{
	sendPositionCommand(0.0,0.0,0.0,0);
}

void SSPP::go2WayPoint()
{
	if(this->enableDrive)
		sendPositionCommand(wayPoint.p.x(),wayPoint.p.y(),wayPoint.phi,1);
	else
		stopDriving();
};

// this function will run in a separate thread
void SSPP::Main()
{
  	pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED,NULL);	
	while(1) 
	{
		pthread_testcancel();     // Thread cancellation point.
	    this->ProcessMessages();
		if(gotNewMap)
		{
			gotMap = false;
			if(this->getMapInfo() < 0)
		    	return;
			if(this->getMap() < 0)
		    	return;
			if(setupPlanner() < 0)
		  	{
		  		puts("Error Happened while initializing Planner!!!");
		  		this->SetError(-1);
		    	return;
		  	}		    
		  	gotNewMap = false; 			
		}
		if(!haveGoal)
		{
			usleep(100000);
			continue;
		}
		if (recievedNewGoal && gotMap)
		{
			recievedNewGoal = false;			
			findPath();
		}
		if(goalReached())
		{
			stopDriving();
		}
		else if(path)
		{
			if (getWayPoint(wayPoint))
			{
				go2WayPoint();
			}
			else
			{
				puts("Can't find a wayPoint for some stupid Reason, using endPose instead");
				wayPoint = endPose;
				go2WayPoint();
			}
		}
		else
		{
				puts("Can't find a path using the Path planner so using reactive Planner instead");
				wayPoint = endPose;
				go2WayPoint();			
		}
		refreshData();
    	usleep(100000);
	}
	pthread_exit(NULL);
}
/*! Forwards the Messeges  from the messege queue to their
 *  specific handler
 */
int SSPP::ProcessMessage(MessageQueue * resp_queue, player_msghdr * hdr, void * data)
{
  	// Forward the Messages
  	switch (hdr->type)
  	{
  		case PLAYER_MSGTYPE_REQ:
	    	return(this->handleConfigs(resp_queue,hdr,data));  			
	    case PLAYER_MSGTYPE_CMD:
	    	return(this->handleCommands(resp_queue,hdr,data));	    	
	    case PLAYER_MSGTYPE_DATA:
	    	return(this->handleData(resp_queue,hdr,data));
	    default:
	    	return -1;
  	}
};

int SSPP::handleData(MessageQueue * resp_queue, player_msghdr * hdr, void * idata)
{
  	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,PLAYER_POSITION2D_DATA_STATE, this->localizer_addr))
  	{
  		player_position2d_data_t* data = reinterpret_cast<player_position2d_data_t*> (idata);
  		currentPose.p.setX(data->pos.px);
  		currentPose.p.setY(data->pos.py);
  		currentPose.phi = data->pos.pa;
//		cout<<"\n	--->>> Odom pose from Position:0 XYTheta=["<<currentPose.p.x()<<"]["<<currentPose.p.y()<<"]["<<currentPose.phi<<"]";  		
		return 0;
  	}
  	else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,PLAYER_PLANNER_DATA_STATE, this->device_addr))
  	{
  		player_planner_data_t * data = reinterpret_cast<player_planner_data_t*> (idata);
  		currentPose.p.setX(data->pos.px);
  		currentPose.p.setY(data->pos.py);
  		currentPose.phi = data->pos.pa;
//		cout<<"\n	--->>> Planner pose from Position:0 XYTheta=["<<currentPose.p.x()<<"]["<<currentPose.p.y()<<"]["<<currentPose.phi<<"]";  		
		return 0;
  	}  	
  	else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,PLAYER_POSITION2D_DATA_STATE, this->position_out_addr))
  	{
  		player_position2d_data_t* data = reinterpret_cast<player_position2d_data_t*> (idata);
  		currentPose.p.setX(data->pos.px);
  		currentPose.p.setY(data->pos.py);
  		currentPose.phi = data->pos.pa;
//		cout<<"\n	--->>> I received this odom pose from Position Interface XYTheta=["<<currentPose.p.x()<<"]["<<currentPose.p.y()<<"]["<<currentPose.phi<<"]";  		
		return 0;
  	}
	else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_DATA,PLAYER_MAP_DATA_INFO, this->map_addr))
  	{
	    if(hdr->size != sizeof(player_map_info_t))
	    {
	    	PLAYER_ERROR("incorrect size for map info");
	      	return(-1);
	    }
    	this->gotNewMap = true;
    	return 0;
  	}  	
  	else
  	{
  		puts("Unknown Data Request !!!");
  	}
  	return -1;
};  
int SSPP::handleConfigs(MessageQueue* resp_queue,player_msghdr * hdr,void * data)
{
	// Is it a new goal for the planner?
  	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,PLAYER_PLANNER_REQ_GET_WAYPOINTS,this->device_addr))
  	{
    	player_planner_waypoints_req_t reply;
    	numPathWayPoints = 0;
    	tempPath=path;
    	while(tempPath)
    	{
    		if((this->numPathWayPoints+1) > PLAYER_PLANNER_MAX_WAYPOINTS)
    			break;
      		reply.waypoints[numPathWayPoints].px = tempPath->pose.p.x();
      		reply.waypoints[numPathWayPoints].py = tempPath->pose.p.y();
      		reply.waypoints[numPathWayPoints].pa = 0.0;
    		numPathWayPoints++;      		    		
    		tempPath= tempPath->next;	
    	}
    	if(this->numPathWayPoints > PLAYER_PLANNER_MAX_WAYPOINTS)
    	{
      		PLAYER_WARN("too many waypoints; truncating list");
      		reply.waypoints_count = 0;
    	}
    	else
      		reply.waypoints_count = this->numPathWayPoints;

	    this->Publish(this->device_addr, resp_queue,PLAYER_MSGTYPE_RESP_ACK,PLAYER_PLANNER_REQ_GET_WAYPOINTS, (void*)&reply, sizeof(reply), NULL);
    	return(0);
  	}
  	// Is it a request to enable or disable the planner?
  	else if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_REQ,PLAYER_PLANNER_REQ_ENABLE,this->device_addr))
  	{
    	if(hdr->size != sizeof(player_planner_enable_req_t))
    	{
      		PLAYER_ERROR("incorrect size for planner enable request");
      		return(-1);
    	}
    	player_planner_enable_req_t* enable_req = (player_planner_enable_req_t*)data;
	    if(enable_req->state)
    	{
    		this->enableDrive = true;
      		PLAYER_MSG0(2,"Robot enabled");
    	}
	    else
    	{
      		this->enableDrive = false;
      		sendPositionCommand(0.0,0.0,0.0,0);
      		PLAYER_MSG0(2,"Robot disabled");
    	}
    	this->Publish(this->device_addr,resp_queue, PLAYER_MSGTYPE_RESP_ACK, PLAYER_PLANNER_REQ_ENABLE);
    	return(0);
  	}  	
  	return -1;
}
int  SSPP::handleCommands(MessageQueue* resp_queue,player_msghdr * hdr,void * data)
{
	if(Message::MatchMessage(hdr, PLAYER_MSGTYPE_CMD,PLAYER_PLANNER_CMD_GOAL,this->device_addr))
  	{
  		player_planner_cmd_t* cmd = (player_planner_cmd_t*)data;
		endPose.p.setX(cmd->goal.px);
  		endPose.p.setY(cmd->goal.py);
  		endPose.phi = cmd->goal.pa;
    	printf("\nnew goal: %f, %f, %f\n", endPose.p.x(), endPose.p.y(), endPose.phi);
    	this->recievedNewGoal = true;
    	this->haveGoal = true;
    	return 0;
  	}
  	return -1;
};
void SSPP::findPath()
{
    path = pathPlanner->findPath(currentPose,endPose,METRIC);
    if(this->debug)
	    pathPlanner->drawPath();	
}
Node * SSPP::closestPathNode(Point location)
{
	Node * all_path = this->path;
	Node * nearest = NULL;
	double dist,shortest= 100000;
	while(all_path)
	{
		dist = Dist(all_path->pose.p,location);
		if(dist < shortest)
		{
			shortest = dist;
			nearest = all_path;
		}
		all_path = all_path->next;
	}
	return nearest;
}

bool SSPP::getWayPoint(Pose &goal)
{
	Node *temp;
	bool retval = false;
	Pose robotLocation = currentPose;
	double angleToWayPoint,prev_angle=0,angle=robotLocation.phi, longestDist=0.2,d,maxAllowedTurn=100;
	temp = closestPathNode(robotLocation.p);
	temp = this->path;
 	while(temp)
 	{
		prev_angle = angle;
		if(temp->next)
	 		angle = ATAN2(temp->next->pose.p,temp->pose.p);
	 	else
	 		angle = prev_angle;
 		d = Dist(robotLocation.p,temp->pose.p);
 		angleToWayPoint = ATAN2(temp->pose.p,robotLocation.p);
 		if( d > traversableDist || d < longestDist)
 		{
 			temp= temp->next;
 			continue;
 		}
		/* 
		 * if we are turning too much starting from this segment
		 * then take the last acceptable goal as the new waypoint.
		 */
		if(fabs(anglediffs(angle,prev_angle)) > DTOR(maxAllowedTurn))
			break;
		d= longestDist;
		goal.p = temp->pose.p;
		goal.phi = prev_angle;
		retval = true;
		temp= temp->next;
 	}
// 	printf("\n longest Visible waypoint Along the Path=%f",longestDist);
 	return retval;
}
void SSPP::sendPositionCommand(double x, double y, double a, unsigned char type)
{
	if(type)
  	{
  		player_position2d_cmd_pos_t posCommand;  	
  		memset(&posCommand,0,sizeof(posCommand));
    	posCommand.pos.px = x;
    	posCommand.pos.py = y;
    	posCommand.pos.pa = a;
    	posCommand.state=1;
    	this->position_device->PutMsg(this->InQueue,PLAYER_MSGTYPE_CMD,PLAYER_POSITION2D_CMD_POS,(void*)&posCommand,sizeof(posCommand),NULL);
  	}
  	else
  	{
    	// velocity control
		player_position2d_cmd_vel_t velCommand;
  		memset(&velCommand,0,sizeof(velCommand));
    	velCommand.vel.px = x;
    	velCommand.vel.py = y;
    	velCommand.vel.pa = a;
    	velCommand.state=1;
    	this->position_device->PutMsg(this->InQueue, PLAYER_MSGTYPE_CMD,PLAYER_POSITION2D_CMD_VEL,(void*)&velCommand,sizeof(velCommand),NULL);
  	}

};

void SSPP::refreshData()
{
	player_planner_data_t data;
	memset(&data,0,sizeof(data));

  	if(path)
    	data.valid = 1;
  	else
    	data.valid = 0;
  	if(path && goalReached())
    	data.done = 1;
  	else
    	data.done = 0;

  	// put the current localize pose
  	data.pos.px = this->currentPose.p.x();
  	data.pos.py = this->currentPose.p.y();
  	data.pos.pa = this->currentPose.phi;
	// put the target goal
  	data.goal.px = this->endPose.p.x();
  	data.goal.py = this->endPose.p.y();
  	data.goal.pa = this->endPose.phi;

  	if(data.valid && !data.done)
  	{
    	data.waypoint.px = this->wayPoint.p.x();
    	data.waypoint.py = this->wayPoint.p.y();
    	data.waypoint.pa = this->wayPoint.phi;
    	data.waypoint_idx = 0;
    	data.waypoints_count = 0;
  	}
  	this->Publish(this->device_addr, NULL,PLAYER_MSGTYPE_DATA,PLAYER_PLANNER_DATA_STATE,(void*)&data,sizeof(data),NULL);
};
 



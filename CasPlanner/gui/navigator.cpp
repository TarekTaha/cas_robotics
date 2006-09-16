#include "navigator.h"
/** @ingroup Components */
/** @{ */
/** @defgroup ComponentNavigator Navigator
 * @brief Navigator Algorithm responsible for navigating a Path

The @p Navigator Model is responsible for navigating a path generated by a local
or a global path planner.

@image html amcl-phe200-0010.jpg " An image"


The @p Navigator Model works as follow :
  - Some thing
  - Something Else

@par Configs

Something else alot of it :) 

@par Provides

- @ref Map : Maybe
- @ref Path : I will see

@par Requires

The @p Navigator Model relies on the following Models:

- "RobotManager" @ref RobotManager : Provides communication with the actual Robot
- @ref MapManager : Provides a laser or an OG map
- "PathPlanner" @ref PathPlanner : path planning model that generates paths from 
a starting point to an end point.

@par Configuration requests

- TODO

@par Configuration file options

- Navigator settings:
  - k_dist (float)
    - Default: 1.8
    - Gain on distance error.
  - k_theta (float)
    - Default: 2.5
    - gain on angular error.
  - safety_dist (float)
    - Default: 8.192 m
    - Closest distance to obstacle before taking action
  - obst_avoid (string)
    - Default: "config-space"
    - the algorithm to be used for obstacle avoidance
  - traversable_dist (float)
    - Default 0.1
    - ???
  - local_dist (float)
    - Default 0.1
    - ???
  - bridge_len (float)
    - Default 0.1
    - ???
  - bridge_res (float)
    - Default 0.1
    - ???
  - reg_grid (float)
    - Default 0.1
    - ???           
  - obst_exp (float)
    - Default 0.1
    - ???             
  - conn_rad (float)
    - Default 0.1
    - ???        
  - obst_pen (float)
    - Default 0.1
    - ???        
  - dist_goal (float)
    - Default 0.1
    - ???        
- Map Settings:
  - pixel_res (float)
    - Default: 0.05
    - Determines the pixel resolution of the underlying map.
- Robot Settings:
  - robot_length (float)
    - Default: 1.2
    - Determines the legth of the Robot.
  - robot_width (float)
    - Default: 0.65
    - Determines the width of the Robot.   
  - robot_model (string)
    - Default: "diff"
    - The Robot motion model.
@par Notes

- Coordinate System:
Global Pixel Coordinate : Follows the image coordinate system where (0,0) is
the upper left corner.
Local Planner Coordinate: where the (0,0) is the center of the laser map
Global Planner Coordinate: where the (0,0) is the center of the map.

@par Example: Using the Navigator Model on the Wheelchair

The following configuration file illustrates the use of the @p Navigator
Model on a wheelchair robot equipped with a SICK LMS200 scanning laser
range finder:

@verbatim
Map
(
  pixel_res 0.028
)
Robot
(
  robot_width  0.65
  robot_height 1.2
  robot_mode   "diff"
)
Navigator
(
  k_dist  1.8
  k_theta 2.5
  safety_dist 0.5
  obst_avoid "config-space"
  traversable_dist 2
  local_dist  2
  bridge_len 2.0
  bridge_res 0.1
  reg_grid 0.3
  obst_exp 0.2
  conn_rad 0.5
  obst_pen 3
  dist_goal 0.2
)
@endverbatim

@author Tarek Taha

@todo
- Add different Motion Models / Currently only differential model is implimented

*/

/** @} */
Navigator::Navigator(PlayGround * playG,RobotManager *r) :
globalPath(NULL),
localPath(NULL),
playGround(playG),
robotManager(r),
local_planner(NULL),
global_planner(r->planningManager)
{
	connect(this, SIGNAL(drawLocalPath(PathPlanner *,Pose *,int *)),robotManager, SLOT(rePaint(PathPlanner*,Pose *,int *)));
}

Navigator::~Navigator()
{
	
}

void Navigator::setupLocalPlanner()
{
	if(!local_planner)
	{
		local_planner = new PlanningManager(robotManager,
											  pixel_res,
											  dist_goal,
											  bridge_len,
											  bridge_res,
											  reg_grid,
											  obst_exp,
											  conn_rad,
											  obst_pen
										   );
	//robotManager->local_planner = local_planner->pathPlanner;							   
	}
}

int Navigator::readConfigs( ConfigFile *cf)
{
	int numSec; 
	numSec = cf->GetSectionCount(); 
	for(int i=0; i < numSec; i++)
	{
	    QString sectionName = cf->GetSectionType(i);
	    if(sectionName == "Navigator")
	    {
		   	obst_avoid    =   cf->ReadString(i, "obst_avoid", "non");
		   	if(obst_avoid == "VFH")
		   	{
		   		this->obstAvoidAlgo = VFH;
		   	}
		   	else if(obst_avoid == "FORCE_FIELD")
		   	{
		   		this->obstAvoidAlgo = FORCE_FIELD;		   		
		   	}
		   	else if(obst_avoid == "CONFIG_SPACE")
		   	{
		   		this->obstAvoidAlgo = CONFIG_SPACE;
		   	}
		   	else if(obst_avoid == "NO_AVOID")
		   	{
		   		this->obstAvoidAlgo = NO_AVOID;		   		
		   	}
		 	k_dist        =   cf->ReadFloat (i, "k_dist", 1.8);
		  	k_theta       =   cf->ReadFloat (i, "k_theta", 2.5);
		  	safety_dist   =   cf->ReadFloat (i, "safety_dist", 0.5);
		  	tracking_dist =   cf->ReadFloat (i, "tracking_dist", 0);
			dist_goal    = 	  cf->ReadFloat (i, "dist_goal", 0.6);
			bridge_len   = 	  cf->ReadFloat (i, "bridge_len", 2);
			bridge_res   = 	  cf->ReadFloat (i, "bridge_res", 0.1);
			reg_grid     = 	  cf->ReadFloat (i, "reg_grid", 0.2);
			obst_exp     = 	  cf->ReadFloat (i, "obst_exp", 0.1);
			conn_rad     = 	  cf->ReadFloat (i, "conn_rad", 0.6);
			obst_pen     = 	  cf->ReadFloat (i, "obst_pen", 3);
			local_dist   = 	  cf->ReadFloat (i, "local_dist", 2);	
			traversable_dist= cf->ReadFloat (i, "traversable_dist", 2);
			linear_velocity = cf->ReadFloat (i, "linear_velocity", 0.1);
			int cnt =	 			cf->GetTupleCount(i,"laser_pose");
			if (cnt != 3)
			{
				cout<<"\n ERROR: Laser Pose should consist of 3 tuples !!!";
				exit(1);
			}
			laser_pose.p.setX(cf->ReadTupleFloat(i,"laser_pose",0 ,0));
			laser_pose.p.setY(cf->ReadTupleFloat(i,"laser_pose",1 ,0));
			laser_pose.phi = (cf->ReadTupleFloat(i,"laser_pose",2 ,0));			

	    }
	    if(sectionName == "Map")
	    {
			pixel_res    = 	  cf->ReadFloat (i, "pixel_res", 0.028);
	    }
		FF = new ForceField(*robotManager->robot,cf);
	}

    qDebug("-> Starting Robot Navigator."); 
    qDebug("*********************************************************************"); 	
  	qDebug("Navigation Parameters:"); 
  	qDebug("\t\t Obstacle Avoidance:\t%s", qPrintable(obst_avoid)); 
    qDebug("\t\t Controller Distance Gain:%f",k_dist); 
  	qDebug("\t\t Controller Theta    Gain:%f",k_theta);
  	qDebug("\t\t Safet Distance :%f",safety_dist);
  	qDebug("\t\t Tracking Distance :%f",tracking_dist);
    qDebug("*********************************************************************"); 
    qDebug("-> Robot Navigator Started.");   	

 	return 1;
}

double Navigator::NearestObstacle(LaserScan laser_scan)
{
	QPointF ray_end,temp[4],intersection;
	Line L1;
	double dist,shortest_dist=10000;
	for(int i=0;i<4;i++)
	{
		temp[i] = robotManager->robot->local_edge_points[i];
	}
	for(int i=0;i<laser_scan.points.size();i++)
	{
		ray_end = Trans2Global(laser_scan.points[i],laser_scan.laserPose);
		for(int j=0;j<4;j++)
		{
			L1.SetStart(temp[j%4]);      L1.SetEnd(temp[(j+1)%4]);
			dist = Dist2Seg(L1,ray_end);
			if(dist < shortest_dist)
			{
				shortest_dist = dist;
			}
		}
	}
	return shortest_dist;	
};

/*!
 * Gets the equivalent area of what u see in ur laser scan from 
 * the Map already stored in the PlanningManager
 */
void Navigator::GenerateLocalMap(QVector<QPointF> laser_scan,Pose laser_pose, Pose rob_location)
{
	double farest_laser_dist = 0,dist, num_pixels;
	QPointF grid_start,temp;
	local_map.clear();
	for(int i=0;i<laser_scan.size();i++)	
	{
		double d = sqrt(pow(laser_scan[i].x(),2)+pow(laser_scan[i].y(),2));
		if( d > farest_laser_dist)
			farest_laser_dist = d;
	}
//	qDebug("Longest Laser Ray = %f",farest_laser_dist);
	dist = sqrt(pow(laser_pose.p.x(),2)+pow(laser_pose.p.y(),2)); 
	num_pixels = (dist + farest_laser_dist + 10)/global_planner->pathPlanner->map->resolution;

 	global_planner->pathPlanner->ConvertToPixel(&rob_location.p);

	grid_start.setX(rob_location.p.x() - num_pixels);
	if(grid_start.x() < 0) 
		grid_start.setX(0);

	grid_start.setY(rob_location.p.y() - num_pixels);
	if(grid_start.y() < 0) 
		grid_start.setY(0);
//    cout<<"\nStart grid: "<<grid_start.x()<<" y:"<<grid_start.y()<<" pixels:"<<num_pixels; fflush(stdout);
	for(int i= (int)(grid_start.x()) ; i< (2*num_pixels + grid_start.x()); i++)
		for(int j=(int)(grid_start.y());j<(2*num_pixels + grid_start.y()); j++)
		{
			if(i<(global_planner->pathPlanner->map->width - 1)  && j<(global_planner->pathPlanner->map->height - 1)) 
				if (global_planner->pathPlanner->map->data[i][j])
				{
					temp.setX(i);
					temp.setY(j);
					global_planner->pathPlanner->ConvertPixel(&temp);
					local_map.push_back(temp);
				}
		}
	return;
};

void Navigator::setPause(bool pause)
{
	this->pause = pause;
}

void Navigator::setObstAvoidAlgo(int algo)
{
    dataLock.lockForWrite();
	this->obstAvoidAlgo = algo;
    dataLock.unlock();
}

int Navigator::getObstAvoidAlgo()
{
	return this->obstAvoidAlgo;
}

bool Navigator::MapModified(QVector<QPointF> laser_scan,Pose rob_location)
{
	local_map_icp.clear();
	laser_scan_icp.clear();
	GenerateLocalMap(laser_scan,Pose(0,0,0),rob_location);
	Geom2D::Point temp;
	for(int i=0;i<local_map.size();i++)
	{
		temp.x = local_map[i].x();
		temp.y = local_map[i].y();
		local_map_icp.push_back(temp);
	}
	for(int i=0;i<laser_scan.size();i++)
	{	
		QPointF t(laser_scan[i].x(),laser_scan[i].y());
		t = Trans2Global(t,rob_location);
		temp.x = t.x();
		temp.y = t.y();
		laser_scan_icp.push_back(temp);
	}
//	qDebug("Local Map size is:%d Laser Scan size:%d",local_map_icp.size(),laser_scan.size());	
	Geom2D::Pose loc;
	loc.p.x = 0;
	loc.p.y = 0;	
	loc.p.laser_index = 0;
	loc.phi = 0;
	delta_pose = icp.align(local_map_icp,laser_scan_icp,loc, 0.5, 10,true);
	if(delta_pose.p.x ==-1 && delta_pose.p.y ==-1 && delta_pose.phi==-1)
	{
		cout <<"\nWARNING: possible misalignment ICP";
		return  true;
	}
	else
	{
		cout <<"\nDelta Pose X:"<<delta_pose.p.x<<" Y:"<<delta_pose.p.y<<" Phi:"<<RTOD(delta_pose.phi);		
		return false;
	}
};

Node * Navigator::closestPathSeg(QPointF location,Node * all_path)
{
//	qDebug("did i segment here ?"); fflush(stdout);
	Node * nearest = NULL;
	double dist,shortest= 100000;
	while(all_path && all_path->next)
	{
		Line l(all_path->pose.p,all_path->next->pose.p);
		dist = Dist2Seg(l,location);
		if(dist < shortest)
		{
			shortest = dist;
			nearest = all_path;
		}
		all_path = all_path->next;
	}
	if (shortest <= 0.02)
	{
		if(nearest->next)
			if(nearest->next->next)
				nearest = nearest->next;
	}
//	qDebug("No not here"); fflush(stdout);	
	return nearest;
}

Node * Navigator::closestPathNode(QPointF location,Node * all_path)
{
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

void Navigator::setPath(Node *p)
{
    dataLock.lockForWrite();
	this->global_path = p;	
    dataLock.unlock();	
}

void Navigator::FollowPath()
{
	return;
}

void Navigator::StopRobot()
{
    dataLock.lockForWrite();
	this->stop_navigating = true;
    dataLock.unlock();		
}

void Navigator::StopNavigating()
{
    dataLock.lockForWrite();
	this->stop_navigating = true;
    dataLock.unlock();			
}
/*!
 * Determines if a Point is in the current laser's free space or not
 */
bool Navigator::inLaserSpace(LaserScan laserScan,Pose robotLocation,QPointF wayPoint)
{
	laserScan.laserPose = Trans2Global(laserScan.laserPose,robotLocation);
	double ang, angle = ATAN2(wayPoint,laserScan.laserPose.p);
	
	for(int i=0;i<laserScan.points.size();i++)
	{
		laserScan.points[i] = Trans2Global(laserScan.points[i],laserScan.laserPose);
		ang = ATAN2(laserScan.points[i],laserScan.laserPose.p);
		if(RTOD(abs(angle-ang))<5)
		{
			if(Dist(laserScan.laserPose.p,laserScan.points[i]) < Dist(laserScan.laserPose.p,wayPoint))
				return false;
		}
	}
	return true;
}

/*!
 * This is the Navigation Thread's main, where the control and the path following takes place.
 */
bool Navigator::getGoal(LaserScan laserScan, Pose &goal)
{
	Node *temp;
	bool retval = false;
	Pose robotLocation = robotManager->robot->robotLocation;
	double angle=0;
	temp = closestPathNode(robotLocation.p,global_path);
 	while(temp && (Dist(robotLocation.p,temp->pose.p) < traversable_dist))
 	{
 		if(temp->next)
			angle = ATAN2(temp->next->pose.p,temp->pose.p); 		
 		if (inLaserSpace(laserScan,robotLocation,temp->pose.p) && (Dist(robotLocation.p,temp->pose.p) > 0.5) )
 		{
 			goal.p = temp->pose.p;
 			goal.phi = angle;
 			retval = true;
 		}
 		// if this is the last node then take it anyways
 		else if((Dist(robotLocation.p,temp->pose.p) < 1) && !temp->next )
 		{
 			goal.p = temp->pose.p;
 			goal.phi = angle; 			
 			retval = true; 			
 		}
 		temp= temp->next;
 	}
 	return retval;
}

void Navigator::run()
{
//	if(robotManager->renderingMethod == PAINTER_2D)
//		connect(this, SIGNAL(pathTraversed()),robotManager->navCon, SLOT(Finished()));	
	if(robotManager->renderingMethod == OPENGL)
	{
//		connect(this, SIGNAL(glRender()),robotManager->->mapViewer, SLOT(update()));	
//		connect(this, SIGNAL(pathTraversed()),playGround->navCon->navControlPanel, SLOT(Finished()));
//		connect(this, SIGNAL(setWayPoint(Pose*)),robotManager->navCon->mapViewer,SLOT(setWayPoint(Pose*)));				
//		connect(this, SIGNAL(renderMapPatch(Map*)),robotManager->navCon->mapViewer,SLOT(renderMapPatch(Map*)));						
	}
	if(robotManager->renderingMethod == OPENGL)
	{
		connect(this, SIGNAL(glRender()),robotManager,SLOT(update()));	
		connect(this, SIGNAL(pathTraversed()),robotManager,SLOT(Finished()));
		connect(this, SIGNAL(setWayPoint(Pose*)),robotManager,SLOT(setWayPoint(Pose*)));				
		connect(this, SIGNAL(renderMapPatch(Map*)),robotManager,SLOT(renderMapPatch(Map*)));						
	}
	QVector <Robot *> availableRobots;
	ControlAction cntrl;
	Timer amcl_timer,delta_timer,redraw_timer,control_timer;
//	double closest_obst=10;
	Pose loc;
	if(!local_planner)
	{
		setupLocalPlanner();
	}
	if(!global_path)
	{
		qDebug("\n --->>> No PATH TO FOLLOW <<<---");
		return ;
	}
	if(!robotManager->commManager)
	{
		qDebug("\t - Communication Manager Not Initialized");
		return;
	}
	if(!robotManager->commManager->connected)
	{
		qDebug("\t - Your not Connected to the Robot, Connect First");
		return;		
	}
	path2Draw = GLOBALPATH;
	redraw_timer.restart();
	control_timer.restart();
	Pose initial_pos;
	initial_pos.p.setX(global_path->pose.p.x());
	initial_pos.p.setY(global_path->pose.p.y());	
	initial_pos.phi = global_path->pose.phi;	
	//Set our Initial Location Estimation
	robotManager->commManager->setLocation(initial_pos);
	sleep(1);
	while(!robotManager->commManager->getLocalized())
	{
		loc = robotManager->commManager->getLocation();
		qDebug("NO Accurate Estimation yet, best current is: x:%f y:%f phi:%f",loc.p.x(),loc.p.y(),RTOD(loc.phi));
		usleep(300000);
	}
	/**********************         Start by the Global Path     ************************/
	path2Follow = global_path;
	/************************************************************************************/
	amcl_timer.restart();	delta_timer.restart();
	// Reset Times
	last_time=0; delta_t=0;	velocity=0;
	end_reached = false;stop_navigating = false;
	double speed,turnRate;
	LaserScan laserScan;
	trail.clear();
	while(!end_reached && !stop_navigating)
	{
		delta_t = delta_timer.secElapsed();
		delta_timer.restart();
		usleep(30000);
		// Get current Robot Location
//		amcl_location = robotManager->commManager->getLocation();
		amcl_location = robotManager->commManager->getOdomLocation();
		EstimatedPos = amcl_location;
		dataLock.lockForWrite();
		robotManager->robot->setPose(amcl_location);
		dataLock.unlock();
		trail.push_back(amcl_location.p);
		
		speed    = robotManager->commManager->getSpeed();
		turnRate =  robotManager->commManager->getTurnRate();		
		// Updating the current Robot Info 
		dataLock.lockForWrite();
		robotManager->robot->setSpeed(speed);
		robotManager->robot->setTurnRate(turnRate);
		dataLock.unlock();
		
		laserScan = robotManager->commManager->getLaserScan();
//		cout<<"\n Current Location X:"<<amcl_location.p.x()<<" Y:"<<amcl_location.p.y()<<" Theta:"<<amcl_location.phi;
		/* If this location is new, then use it. Otherwise
		 * estimate the location based on the last reading.
		 */
//		if(old_amcl != amcl_location)
//		{
//			// Recording the last time Data changed
//			last_time = amcl_timer.secElapsed();
//			qDebug("last Time amcl Change took:%f",last_time);
//			// resetting the timer
//			amcl_timer.restart(); 
//			// Override the Estimated Location with the AMCL hypothesis
//			old_amcl.p.setX(amcl_location.p.x()); EstimatedPos.p.setX(amcl_location.p.x());
//			old_amcl.p.setY(amcl_location.p.y()); EstimatedPos.p.setY(amcl_location.p.y());
//			old_amcl.phi = EstimatedPos.phi = amcl_location.phi;
//		}
//		else
//		{
//			// Estimate the location based on the Last AMCL location and the vehichle model
//			EstimatedPos.phi += robotManager->commManager->getTurnRate()*delta_t;
//			EstimatedPos.p.setX(EstimatedPos.p.x() + velocity*cos(EstimatedPos.phi)*delta_t);
//			EstimatedPos.p.setY(EstimatedPos.p.y() + velocity*sin(EstimatedPos.phi)*delta_t);
//			//cout<<"\nVelocity is:"<<velocity<<" Side Speed is:"<<pp->SideSpeed();
//			if (velocity!= speed) //	Velocity Changed?
//				velocity = speed;
//			//cout<<"\n New data arrived Velocity="<<pp->Speed()<<" Angular"<<pp->SideSpeed();
//		}
		/* if we were following a local path and crossed the boundaried of the local
		 * area without reaching the local destination then go back to the global path
		 */
//		if (path2Follow == local_path)
//		{
//			if(Dist(EstimatedPos.p,local_planner->pathPlanner->map->global_pose.p)>local_dist)
//			{
//				local_planner->pathPlanner->FreeResources();
//				path2Follow = global_path;
//				path2Draw = GLOBALPATH;
//			}
//		}
		//first = ClosestPathSeg(EstimatedPos.p,path2Follow);
		//qDebug("Robot Pose x:%f y:%f phi%f",EstimatedPos.p.x(),EstimatedPos.p.y(),EstimatedPos.phi);		
		fflush(stdout);
		first = closestPathSeg(EstimatedPos.p,global_path);
		if(!first)
		{
			qDebug("Path Doesn't contain any segment to follow !!!");
			break;
		}
		// Is it the last Segment ?
		if (first->next)
		if (!first->next->next)
		{
			if(Dist(first->next->pose.p,EstimatedPos.p)<=0.4)
			{
				if (local_planner->pathPlanner->path)
				{
					qDebug("--->>> Local Path Traversed !!!");					
					local_planner->pathPlanner->FreePath();
					path2Follow = global_path;
					usleep(100000);
					continue;
				}
				else
				{
					qDebug("--->>> Destination Reached !!!");
					emit pathTraversed();
		 			end_reached = true;
					break;
				}
			}
		}
		last  = first->next;	ni = first->pose.p;
		SegmentStart.setX(ni.x());	SegmentStart.setY(ni.y());
		ni = last->pose.p;  SegmentEnd.setX(ni.x());  SegmentEnd.setY(ni.y());	
		direction = -1;
		angle = atan2(SegmentEnd.y() - SegmentStart.y(),SegmentEnd.x() - SegmentStart.x());

		/* If we chose to follow a virtual point on the path then calculate that point
		 * It will not be used most of the time, but it adds accuracy in control for
		 * long line paths.
		 */
		//qDebug("Vel =%.3f m/sev X=[%.3f] Y=[%.3f] Theta=[%.3f] time=%g",speed,EstimatedPos.p.x(),EstimatedPos.p.y(),RTOD(EstimatedPos.phi),delta_t);
		tracking_point.setX(EstimatedPos.p.x() + tracking_dist*cos(EstimatedPos.phi) - 0*sin(EstimatedPos.phi));
		tracking_point.setY(EstimatedPos.p.y() + tracking_dist*sin(EstimatedPos.phi) + 0*cos(EstimatedPos.phi)); 

		// Distance to the path Segment
		distance = Dist(SegmentEnd,tracking_point);
		Line l(SegmentStart,SegmentEnd);
		displacement = Dist2Seg(l,tracking_point);

		//qDebug("First X[%.3f]Y[%.3f] Last=X[%.3f]Y[%.3f] Target Angle =[%.3f] Cur_Ang =[%.3f]", SegmentStart.x(),SegmentStart.y() ,SegmentEnd.x(),SegmentEnd.y() ,RTOD(angle),RTOD(EstimatedPos.phi));
		//qDebug("Displ=[%.3f] Dist to Segend=[%.3f] D-Next=[%.3f]",displacement ,distance,distance_to_next);
		/* If we are too close to obstacles then let the local planner takes control
		 * steps :
		 * 1- Takes a local laser Scan from the server.
		 * 2- Build a local occupancy grid based on the laser scan.
		 * 3- Give the generated map to the local planner.
		 * 4- Plan a path from the current location to a point
		 *    on the global path that is X distance away
		 * 5- Follow that path
		 */

//		QTime local_planning_time,icp_time;
//		closest_obst = NearestObstacle(laserScan);
//		qDebug("Closest Distance to Obstacles is:%f Saftey Dist:%f",closest_obst,sf);
//		if(closest_obst < safety_dist)
//		{
//			icp_time.restart();
//			// If we don't already have a local map or the local environment is changed then re-plan
//			if ( !local_planner->pathPlanner->path) //|| MapModified(laser_set,EstimatedPos))
//			{
//				//qDebug("Icp Check took:%d msec",icp_time.elapsed());					
//				//Stop the Robot before planning
//				robotManager->commManager->setSpeed(0);
//				robotManager->commManager->setTurnRate(0);
//				
//				// local Planning Map Distance
//				double target_angle;
//			 	Pose start,target,loc, pixel_loc = EstimatedPos;
//			 	loc = pixel_loc; target.phi = 0;
//			 	
//			 	// Current Locaion in the Global coordinate
//			 	global_planner->pathPlanner->ConvertToPixel(&pixel_loc.p);
////				qDebug("Location Global Metric X:%f Y:%f",loc.p.x(),loc.p.y());
////				qDebug("Location Global Pixel  X:%f Y:%f",pixel_loc.p.x(),pixel_loc.p.y());
//	
//				local_planning_time.restart();
//			 	local_planner->SetMap(laserScan,local_dist,EstimatedPos);
//				local_planner->GenerateSpace();
//			 	Node * temp;
//			 	temp = global_path;
//			 	if(!temp)
//			 	{
//			 		qDebug("Global Path is empty, Something went wrong, Emergency Stopping ");
//					robotManager->commManager->setSpeed(0);
//					robotManager->commManager->setTurnRate(0);
//					sleep(1);
//					exit(1);
//			 	}
//				 //Find the farest way Point on the global path
//				 //* that belongs to the local map
//				 
//				double dist=0;
//				temp = ClosestPathSeg(EstimatedPos.p,global_path);
//			 	while(temp->next && dist < traversable_dist)
//			 	{
//				 	QPointF boundary_check;
//		 			//traversable_dist += Dist(temp->pose.p,temp->next->pose.p);
//					target_angle = ATAN2(temp->next->pose.p,temp->pose.p);
//		 			dist+= Dist(temp->pose.p,temp->next->pose.p);
//				 	boundary_check.setX(temp->pose.p.x());
//				 	boundary_check.setY(temp->pose.p.y());
////					qDebug("Target Global Metric X:%f Y:%f",boundary_check.x(),boundary_check.y());
//				 					 				 	
//				 	// Transfer to the global Pixel Coordinate
//				 	global_planner->pathPlanner->ConvertToPixel(&boundary_check);				 	
////					qDebug("Target Pixel Global  X:%f Y:%f",boundary_check.x(),boundary_check.y());
//									 	
//				 	// Transfer to the local Pixel Coordinate
//				 	boundary_check.setX(boundary_check.x() - pixel_loc.p.x() + local_planner->pathPlanner->map->center.x());	
//				 	boundary_check.setY(boundary_check.y() - pixel_loc.p.y() + local_planner->pathPlanner->map->center.y());				 	
////					qDebug("Target Pixel Local   X:%f Y:%f",boundary_check.x(),boundary_check.y());
//	
//					// Check Boundaries
//				 	if (boundary_check.x() < 0 || boundary_check.y() < 0 )
//					 	break;
//				 	if (boundary_check.x() > (local_planner->pathPlanner->map->width - 1) || boundary_check.y() > (local_planner->pathPlanner->map->height - 1))
//				 		break;
//				 	// This is a valid target in the local Map
//					target.p.setX(boundary_check.x());	 		
//					target.p.setY(boundary_check.y());	 							
//				 	target.phi = target_angle;	
////					qDebug("Target Local Pixel X:%f Y:%f",target.p.x(),target.p.y());
//			 		temp= temp->next;
//			 	}
//				
//				 /*Search Start location is the center of the Robot			
//				 * currently it's the center of the laser, this will have
//				 * to be modified to translate the laser location to the 
//				 * robot's coordinate : TODO
//				 */
//				 
//				 
//			 	start.p.setX(local_planner->pathPlanner->map->center.x());
//			 	start.p.setY(local_planner->pathPlanner->map->center.y()); 	
//			 	start.phi = EstimatedPos.phi;
//			 	
////				qDebug("Start Local Pixel  X:%f Y:%f",start.p.x(),start.p.y());			 	
////				qDebug("Target Local Pixel X:%f Y:%f",target.p.x(),target.p.y());
//	
//			 	local_path = local_planner->FindPath(start,target);
//			 	if (local_path)
//			 	{
//			 		Node * temp = local_path;	
//			 		// Changing the Path to the Global Coordinate
//			 		pixel_loc.p -= local_planner->pathPlanner->map->center;
//			 		while(temp)
//			 		{
//			 			local_planner->pathPlanner->ConvertToPixel(&temp->pose.p);
//						temp->pose.p += pixel_loc.p;
//			 			global_planner->pathPlanner->ConvertPixel(&temp->pose.p);
//			 			temp = temp->next;
//			 		}
//			 		qDebug("Local Path found");
//			 		path2Follow = local_path;
//			 		path2Draw = LOCALPATH;
//			 		continue;
//			 	}
//			 	else
//			 	{
//			 		path2Follow = global_path;
//			 		path2Draw = GLOBALPATH;
//			 	}
//		 		qDebug("Local Planning took %dms",local_planning_time.elapsed());	
//			}
//		}
 		if (redraw_timer.msecElapsed()>100)
 		{
//			if(this->mapPatch)
//				delete mapPatch;
//			mapPatch = mapManager.provideLaserOG(laserScan,2.0,0.05,EstimatedPos);	
			if(robotManager->renderingMethod == PAINTER_2D) 			
		 		emit drawLocalPath(local_planner->pathPlanner,&loc,&path2Draw);	
			else if(robotManager->renderingMethod == OPENGL)		 			
			{
			 	emit glRender();
//			 	emit renderMapPatch(mapPatch);
//			 	emit renderMapPatch(robotManager->planner->pathPlanner->map);
			}
		 	redraw_timer.restart();	
		}
		/* Get the control Action to be applied, in this case it's a
		 * simple linear control. It's accurate enough for traversing 
		 * the generated paths.
		 */
//		Pose goal(SegmentEnd.x(),SegmentEnd.y(),angle);
		Pose goal;
		if(!getGoal(laserScan,goal))
		{
//			robotManager->commManager->setSpeed(0);
//			robotManager->commManager->setTurnRate(0);			
//			robotManager->planningManager->setStart(robotManager->robot->robotLocation);
//			//TODO: ICP with the local Area and give the Corrected Location to the function Below
//			robotManager->planningManager->updateMap(laserScan,local_dist,robotManager->robot->robotLocation);
//			robotManager->planningManager->findPath(METRIC);
//			continue;
		}
		else
		{
			wayPoint = goal;
		}
//		emit setWayPoint(&goal);

		QTime ff_time;
		if(!pause)
		{
			switch(obstAvoidAlgo)		 
			{
				case FORCE_FIELD:
					availableRobots.clear();
					for(int i=0;i<playGround->robotPlatforms.size();i++)
					{
						if(playGround->robotPlatforms[i]!= robotManager)
							availableRobots.push_back(playGround->robotPlatforms[i]->robot);
					}
					//Force Field
					velVector action;
					ff_time.restart();
					//qDebug("================================= FORCE FIELD STARTS ===============================");
					qDebug("Current Robot      --->>> Turn Rate:%lf and Speed is:%lf Delta Time:%lf",turnRate,speed,delta_t);
				 	qDebug("Current Robot Pose --->>> x:%f y:%f phi:%f",amcl_location.p.x(),amcl_location.p.y(),RTOD(amcl_location.phi));
				 	control_timer.restart();
					action = FF->GenerateField(amcl_location,laserScan,wayPoint,speed,turnRate,availableRobots,delta_t);
//					qDebug("Force Field Returned     --->>> Speed is:%f TurnRate is:%f  time to calculate FF is:%dms Loop Delta_t:%fsec",action.speed,action.turnRate,ff_time.elapsed(),delta_t);	
					robotManager->commManager->setSpeed(action.speed);						
					robotManager->commManager->setTurnRate(action.turnRate);		
					control_timer.restart();				
					//qDebug("================================= FORCE FIELD ENDS  ===============================\n");					
					break;		
				case CONFIG_SPACE:
					break;
				case NO_AVOID:
					// Linear Controller 
					cntrl = getAction(EstimatedPos.phi,angle,displacement,path2Follow->direction,linear_velocity);
					qDebug("Control Action Linear:%f Angular:%f",path2Follow->direction*cntrl.linear_velocity,
					cntrl.angular_velocity);
					/* Angular Velocity Thrusholded, just trying not to
					 * exceed the accepted limits. Or setting up a safe 
					 * turn speed.
					 */
					if(cntrl.angular_velocity >   0.2)
						cntrl.angular_velocity =  0.2;
					if(cntrl.angular_velocity <  -0.2)
						cntrl.angular_velocity = -0.2;
					if(log)
						fprintf(file,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %g %g\n",EstimatedPos.p.x(),EstimatedPos.p.y(),amcl_location.p.x(), amcl_location.p.y(), displacement ,error_orientation ,cntrl.angular_velocity,SegmentStart.x(),SegmentStart.y(),SegmentEnd.x(),SegmentEnd.y(),delta_timer.secElapsed(),last_time);			
					//Normal Linear Follower
					robotManager->commManager->setSpeed(path2Follow->direction*cntrl.linear_velocity);
					robotManager->commManager->setTurnRate(cntrl.angular_velocity);							
					break;
				case VFH:
					// Vector Field Histogram
//					qDebug("Sending to VFH goto X:%f Y:%f Phi%f",goal.p.x(),goal.p.y(),goal.phi);
					robotManager->commManager->vfhGoto(wayPoint);	
					break;		
				default:
					qDebug("Unknown Obstacle Avoidance Algorithm used !!!");
			}	
		}
		else
		{
			robotManager->commManager->setSpeed(0);
			robotManager->commManager->setTurnRate(0);
		}
		loc = EstimatedPos;
	}
	robotManager->commManager->setSpeed(0);
	robotManager->commManager->setTurnRate(0);
	local_planner->pathPlanner->FreeResources();
	qDebug("Thread Loop Terminated Normally !!!");
	return;
}


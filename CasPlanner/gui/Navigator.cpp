#include "Navigator.h"

Navigator::Navigator(RobotManager *r) :
globalPath(NULL),
localPath(NULL),
robotManager(r),
local_planner(NULL)
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
		local_planner = new PlanningManager(
											  robot_length,
											  robot_width,
											  robot_model,
											  rotation_center,
											  pixel_res,
											  dist_goal,
											  bridge_len,
											  bridge_res,
											  reg_grid,
											  obst_exp,
											  conn_rad,
											  obst_pen
										   );
	robotManager->local_planner = local_planner->pathPlanner;							   
	}
}

int Navigator::config( ConfigFile *cf, int sectionid)
{
   	obst_avoid    =   cf->ReadString(sectionid, "obst_avoid", "non");
 	k_dist        =   cf->ReadFloat (sectionid, "k_dist", 1.8);
  	k_theta       =   cf->ReadFloat (sectionid, "k_theta", 2.5);
  	safety_dist   =   cf->ReadFloat (sectionid, "safety_dist", 0.5);
  	tracking_dist =   cf->ReadFloat (sectionid, "tracking_dist", 0);
	
	robot_length = 	  cf->ReadFloat (sectionid, "robot_length", 1.2);
	robot_width  = 	  cf->ReadFloat (sectionid, "robot_width", 0.65);
	pixel_res    = 	  cf->ReadFloat (sectionid, "pixel_res", 0.05);
	dist_goal    = 	  cf->ReadFloat (sectionid, "dist_goal", 0.6);
	bridge_len   = 	  cf->ReadFloat (sectionid, "bridge_len", 2);
	bridge_res   = 	  cf->ReadFloat (sectionid, "bridge_res", 0.1);
	reg_grid     = 	  cf->ReadFloat (sectionid, "reg_grid", 0.2);
	obst_exp     = 	  cf->ReadFloat (sectionid, "obst_exp", 0.1);
	conn_rad     = 	  cf->ReadFloat (sectionid, "conn_rad", 0.4);
	obst_pen     = 	  cf->ReadFloat (sectionid, "obst_pen", 3);
	local_dist   = 	  cf->ReadFloat (sectionid, "local_dist", 2);	
	traversable_dist= cf->ReadFloat (sectionid, "traversable_dist", 2);
	linear_velocity = cf->ReadFloat (sectionid, "linear_velocity", 0.1);
	robot_model  =    cf->ReadString(sectionid, "robot_mode", "diff");
	rotation_center.setX(0);
	rotation_center.setY(-0.3);

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

double Navigator::NearestObstacle(QVector<QPointF> laser_scan,Pose pose)
{
	QPointF ray_end,temp[4],intersection;
	Line L1,L2;
	double dist,shortest_dist=10000;
	for(int i=0;i<4;i++)
	{
		temp[i] = Trans2Global(local_planner->pathPlanner->local_edge_points[i],pose);
	}
	for(int i=0;i<laser_scan.size();i++)
	{
		for(int j=0;j<4;j++)
		{
			ray_end = Trans2Global(laser_scan[i],pose);
			L1.SetStart(temp[j%4]);      L1.SetEnd(temp[(j+1)%4]);
			L2.SetStart(pose.p);         L2.SetEnd(ray_end);
			if(LineInterLine(L1,L2,intersection))
			{
				dist = Dist(intersection,ray_end);
				if(dist < shortest_dist)
				{
					shortest_dist = dist;
					//qDebug("Shortest:%f",shortest_dist);
				}
			}
		}
	}
	return shortest_dist;	
}
Node * Navigator::FindClosest(QPointF location,Node * all_path)
{
//	QTime timer;
//	timer.restart();
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
//	qDebug("It took:%dms",timer.elapsed());
	return nearest;
}

void Navigator::setPath(Node *p)
{
	this->global_path = p;	
}

void Navigator::FollowPath()
{
	return;
}

void Navigator::run()
{
	ControlAction cntrl;
	QTime amcl_timer,delta_timer,redraw_timer;
	Pose loc;
	int counter=0;
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
	path2Draw = SHOWGLOBALPATH;
	redraw_timer.restart();
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
	amcl_timer.start();
	delta_timer.start();
	// Reset Times
	last_time=0; delta_t=0;	velocity=0;
	end_reached = false;
	double sf = safety_dist;
	while(!end_reached)
	{
		delta_t = delta_timer.elapsed()/1e3;
		delta_timer.restart();
		usleep(10000);
		first = FindClosest(loc.p,path2Follow);
		// Is it the last Segment ?
		if (!first->next->next)
		{
			if(Dist(first->next->pose.p,EstimatedPos.p)<=0.2)
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
		 			end_reached = true;
					break;
				}
			}
		}
		last  = first->next;	ni = first->pose.p;
		SegmentStart.setX(ni.x());	SegmentStart.setY(ni.y());
//		qDebug("--->>>NEW Line SEG Starts x[%.3f]y[%.3f]",SegmentStart.x(),SegmentStart.y());
		ni = last->pose.p;  SegmentEnd.setX(ni.x());  SegmentEnd.setY(ni.y());	
//		qDebug("--->>>Ends at   x[%.3f]y[%.3f] <<<---",SegmentEnd.x(),SegmentEnd.y());
		direction = -1;
		angle = atan2(SegmentEnd.y() - SegmentStart.y(),SegmentEnd.x() - SegmentStart.x());
//		qDebug("--->>> Orientation(Planned) to follow :=%.3f <<<---",RTOD(angle));

		// Estimate the location based on the Last AMCL location and the vehichle model
		EstimatedPos.phi += robotManager->commManager->getTurnRate()*delta_t;
		EstimatedPos.p.setX(EstimatedPos.p.x() + velocity*cos(EstimatedPos.phi)*delta_t);
		EstimatedPos.p.setY(EstimatedPos.p.y() + velocity*sin(EstimatedPos.phi)*delta_t);
		//cout<<"\nVelocity is:"<<velocity<<" Side Speed is:"<<pp->SideSpeed();
		if (velocity!= robotManager->commManager->getSpeed()) //	Velocity Changed?
			velocity = robotManager->commManager->getSpeed();
		//cout<<"\n New data arrived Velocity="<<pp->Speed()<<" Angular"<<pp->SideSpeed();
		
		// Get current Robot Location
		amcl_location = robotManager->commManager->getLocation();
		/* Is it a new hypothesis (not the same as the last)
		 * so override the estimation based on the robot model
		 * with the AMCL localizer's estimation
		 */
		if(old_amcl != amcl_location)
		{
			// Recording the last time Data changed
			last_time = amcl_timer.elapsed()/1e3;
			// resetting the timer
			amcl_timer.restart(); 
			// Override the Estimated Location with the AMCL hypothesis
			old_amcl.p.setX(amcl_location.p.x()); EstimatedPos.p.setX(amcl_location.p.x());
			old_amcl.p.setY(amcl_location.p.y()); EstimatedPos.p.setY(amcl_location.p.y());
			old_amcl.phi = EstimatedPos.phi = amcl_location.phi;
		}
		/* If we chose to follow a virtual point on the path then calculate that point
		 * It will not be used most of the time, but it adds accuracy in control for
		 * long line paths.
		 */
		//qDebug("Vel =%.3f m/sev X=[%.3f] Y=[%.3f] Theta=[%.3f] time=%g",robotManager->commManager->getSpeed(),EstimatedPos.p.x(),EstimatedPos.p.y(),RTOD(EstimatedPos.phi),delta_t);
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
		QTime local_planning_time;
		double closest_obst =NearestObstacle(robotManager->commManager->getLaserScan(0),robotManager->commManager->getLocation());
		qDebug("Closest Distance to Obstacles is:%f Saftey Dist:%f",closest_obst,sf);
//		if(robotManager->commManager->getClosestObst() < sf)
		if(closest_obst < sf && !local_planner->pathPlanner->path)
		{
			// local Planning Map Distance
			double target_angle;
		 	Pose start,target,loc, pixel_loc = robotManager->commManager->getLocation();
		 	loc = pixel_loc; target.phi = 0;
		 	
		 	// Current Locaion in the Global coordinate
		 	robotManager->planner->pathPlanner->ConvertToPixel(&pixel_loc.p);
//			qDebug("Location Global Metric X:%f Y:%f",loc.p.x(),loc.p.y());
//			qDebug("Location Global Pixel  X:%f Y:%f",pixel_loc.p.x(),pixel_loc.p.y());
			
			local_planning_time.restart();
		 	local_planner->SetMap(robotManager->commManager->getLaserScan(0),local_dist,robotManager->commManager->getLocation());
			local_planner->GenerateSpace();
		 	Node * temp;
		 	temp = global_path;
		 	if(!temp)
		 	{
		 		qDebug("Global Path is empty, Something went wrong, Emergency Stopping ");
				robotManager->commManager->setSpeed(0);
				robotManager->commManager->setTurnRate(0);
				sleep(1);
				exit(1);
		 	}
			/* Find the farest way Point on the global path
			 * that belongs to the local map
			 */
			double dist=0;
			temp = FindClosest(EstimatedPos.p,global_path);
		 	while(temp->next && dist < traversable_dist)
		 	{
			 	QPointF boundary_check;
	 			//traversable_dist += Dist(temp->pose.p,temp->next->pose.p);
				target_angle = ATAN2(temp->next->pose.p,temp->pose.p);
	 			dist+= Dist(temp->pose.p,temp->next->pose.p);
			 	boundary_check.setX(temp->pose.p.x());
			 	boundary_check.setY(temp->pose.p.y());
//				 	qDebug("Target Global Metric X:%f Y:%f",boundary_check.x(),boundary_check.y());
			 					 				 	
			 	// Transfer to the global Pixel Coordinate
			 	robotManager->planner->pathPlanner->ConvertToPixel(&boundary_check);				 	
//					qDebug("Target Pixel Global  X:%f Y:%f",boundary_check.x(),boundary_check.y());
								 	
			 	// Transfer to the local Pixel Coordinate
			 	boundary_check.setX(boundary_check.x() - pixel_loc.p.x() + local_planner->pathPlanner->map->center.x());	
			 	boundary_check.setY(boundary_check.y() - pixel_loc.p.y() + local_planner->pathPlanner->map->center.y());				 	
//					qDebug("Target Pixel Local   X:%f Y:%f",boundary_check.x(),boundary_check.y());

				// Check Boundaries
			 	if (boundary_check.x() < 0 || boundary_check.y() < 0 )
				 	break;
			 	if (boundary_check.x() > (local_planner->pathPlanner->map->width - 1) || boundary_check.y() > (local_planner->pathPlanner->map->height - 1))
			 		break;
			 	// This is a valid target in the local Map
				target.p.setX(boundary_check.x());	 		
				target.p.setY(boundary_check.y());	 							
			 	target.phi = target_angle;	
//					qDebug("Target Local Pixel X:%f Y:%f",target.p.x(),target.p.y());
		 		temp= temp->next;
		 	}
			
			/* Search Start location is the center of the Robot			
			 * currently it's the center of the laser, this will have
			 * to be modified to translate the laser location to the 
			 * robot's coordinate : TODO
			 */
			 
		 	start.p.setX(local_planner->pathPlanner->map->center.x());
		 	start.p.setY(local_planner->pathPlanner->map->center.y()); 	
		 	start.phi = EstimatedPos.phi;
		 	
//				qDebug("Start Local Pixel  X:%f Y:%f",start.p.x(),start.p.y());			 	
//				qDebug("Target Local Pixel X:%f Y:%f",target.p.x(),target.p.y());

		 	local_path = local_planner->FindPath(start,target);
		 	if (local_path)
		 	{
		 		Node * temp = local_path;
		 		pixel_loc.p -= local_planner->pathPlanner->map->center;
		 		while(temp)
		 		{
		 			local_planner->pathPlanner->ConvertToPixel(&temp->pose.p);
					temp->pose.p += pixel_loc.p;
		 			robotManager->planner->pathPlanner->ConvertPixel(&temp->pose.p);
		 			temp = temp->next;
		 		}
		 		qDebug("Local Path found");
		 		path2Follow = local_path;
		 	}
		 	else
		 	{
		 		sf = safety_dist;
		 		path2Follow = global_path;
		 	}
	 		qDebug("Local Planning took %dms",local_planning_time.elapsed());	
	 		path2Draw = SHOWLOCALPATH;
		 	emit drawLocalPath(local_planner->pathPlanner,&loc,&path2Draw);
		}
		else
		{
			if(!local_planner->pathPlanner->path)
			{
				path2Follow = global_path;
		 		path2Draw = SHOWGLOBALPATH;
		 		if (redraw_timer.elapsed()>100)
		 		{
				 	emit drawLocalPath(local_planner->pathPlanner,&loc,&path2Draw);		
				 	redraw_timer.restart();	
		 		}
			}
		}
		/* Get the control Action to be applied, in this case it's a
		 * simple linear control. It's accurate enough for traversing 
		 * the generated paths.
		 */
		cntrl = getAction(EstimatedPos.phi,angle,displacement,path2Follow->direction,linear_velocity);
		//qDebug("Control Action Linear:%f Angular:%f",path2Follow->direction*cntrl.linear_velocity,
		//cntrl.angular_velocity);
		/* Angular Velocity Thrusholded, just trying not to
		 * exceed the accepted limits. Or setting up a safe 
		 * turn speed.
		 */
		if(cntrl.angular_velocity >   0.2)
			cntrl.angular_velocity =  0.2;
		if(cntrl.angular_velocity <  -0.2)
			cntrl.angular_velocity = -0.2;			
		if(log)
			fprintf(file,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %g %g\n",EstimatedPos.p.x(),EstimatedPos.p.y(),amcl_location.p.x(), amcl_location.p.y(), displacement ,error_orientation ,cntrl.angular_velocity,SegmentStart.x(),SegmentStart.y(),SegmentEnd.x(),SegmentEnd.y(),delta_timer.elapsed()/1e3,last_time);
		robotManager->commManager->setSpeed(path2Follow->direction*cntrl.linear_velocity);
		robotManager->commManager->setTurnRate(cntrl.angular_velocity);				

		loc = EstimatedPos;
	}
	robotManager->commManager->setSpeed(0);
	robotManager->commManager->setTurnRate(0);
	return;
}


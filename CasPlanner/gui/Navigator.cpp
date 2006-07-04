#include "Navigator.h"

Navigator::Navigator()
{
}

Navigator::~Navigator()
{
}

int Navigator::config( ConfigFile *cf, int sectionid)
{
   	obst_avoid  =   cf->ReadString(sectionid, "obst_avoid", "non");
 	k_dist      =   cf->ReadFloat (sectionid, "k_dist", 1.8);
  	k_theta     =   cf->ReadFloat (sectionid, "k_theta", 2.5);
  	safety_dist =   cf->ReadFloat (sectionid, "safety_dist", 0.1);
  	tracking_dist =   cf->ReadFloat (sectionid, "tracking_dist", 0);
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

void Navigator::setCommManager(CommManager* com)
{
	this->commManager = com;
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
	QTime timer2,delta_timer;
	Pose loc;
	if(!global_path)
	{
		qDebug("\n --->>> No PATH TO FOLLOW <<<---");
		return ;
	}
	if(!commManager)
	{
		qDebug("\t - Communication Manager Not Initialized");
		return;
	}
	if(!commManager->connected)
	{
		qDebug("\t - Your not Connected to the Robot, Connect First");
		return;		
	}
	while(!commManager->getLocalized())
	{
		loc = commManager->getLocation();
		qDebug("Accurate Estimation Not found yet, best current is: x:%f y:%f phi:%f",loc.p.x(),loc.p.y(),RTOD(loc.phi));
		usleep(300000);
	}
	/********************** Start from the root and move forward ************************/
	first 		= global_path;
	prev_angle	= global_path->angle;
	global_path = global_path->next;
	/************************************************************************************/
	timer2.start();
	delta_timer.start();
	//Reset Times
	last_time=0; delta_t=0;	velocity=0;
	while(!end_reached) 
	{
		timer2.restart();
		delta_timer.restart();
		last = global_path;
		ni = first->location; 
		SegmentStart.setX(ni.x()); 
		SegmentStart.setY(ni.y());
		qDebug("--->>>NEW Line SEG Starts x[%.3f]y[%.3f]",SegmentStart.x(),SegmentStart.y());
		ni = last->location;  SegmentEnd.setX(ni.x());  SegmentEnd.setY(ni.y());	
		qDebug("--->>>Ends at   x[%.3f]y[%.3f] <<<---",SegmentEnd.x(),SegmentEnd.y());
		direction = -1;
		angle = atan2(SegmentEnd.y() - SegmentStart.y(),SegmentEnd.x() - SegmentStart.x());
		qDebug("--->>> Orientation(Planned) to follow :=%.3f <<<---",RTOD(angle));
		segment_navigated = false;
		while (!segment_navigated && !stop_following) // Loop for each global_path segment
		{
			// Estimate the location based on the Last AMCL location and the vehichle model
			delta_t = delta_timer.elapsed()/1e3;
			EstimatedPos.phi += commManager->getTurnRate()*delta_t;
			EstimatedPos.p.setX(EstimatedPos.p.x() + velocity*cos(EstimatedPos.phi)*delta_t);
			EstimatedPos.p.setY(EstimatedPos.p.y() + velocity*sin(EstimatedPos.phi)*delta_t);
			//cout<<"\nVelocity is:"<<velocity<<" Side Speed is:"<<pp->SideSpeed();
			if (velocity!= commManager->getSpeed()) //	Velocity Changed?
				velocity = commManager->getSpeed();
			//cout<<"\n New data arrived Velocity="<<pp->Speed()<<" Angular"<<pp->SideSpeed();
			// Get current Robot Location
			amcl_location = commManager->getLocation();
			// Is it a new hypothesis (not the same as the last)?
			if(old_amcl.p.x() != amcl_location.p.x() || old_amcl.p.y() !=amcl_location.p.y() )
			{
				// Recording the last time Data changed
				last_time = timer2.elapsed()/1e3;
				// resetting the timer
				timer2.restart(); 
				// Override the Estimated Location with the AMCL hypothesis
				old_amcl.p.setX(amcl_location.p.x()); EstimatedPos.p.setX(amcl_location.p.x());
				old_amcl.p.setY(amcl_location.p.y()); EstimatedPos.p.setY(amcl_location.p.y());
				EstimatedPos.phi = amcl_location.phi;
			}
			// Determine the Tracking Point 
			qDebug("Vel =%.3f m/sev X=[%.3f] Y=[%.3f] Theta=[%.3f] time=%g",commManager->getSpeed(),EstimatedPos.p.x(),EstimatedPos.p.y(),RTOD(EstimatedPos.phi),delta_t);
			tracking_point.setX(EstimatedPos.p.x() + tracking_dist*cos(EstimatedPos.phi) - 0*sin(EstimatedPos.phi));
			tracking_point.setY(EstimatedPos.p.y() + tracking_dist*sin(EstimatedPos.phi) + 0*cos(EstimatedPos.phi)); 
			// Distance to the path Segment
			distance = Dist(SegmentEnd,tracking_point);
			displacement = DistToLineSegment(SegmentStart,SegmentEnd,tracking_point);
			// Did we reach the last segment ???
			if (global_path->next) // NO we didnt
			{
				QPointF n;
				n.setX(global_path->next->location.x());
				n.setY(global_path->next->location.y());
				distance_to_next = DistToLineSegment(SegmentEnd,n,tracking_point);
			}
			else // YES this is the last segment
			{
				distance_to_next = 100;
				if (distance <= 0.3)
					segment_navigated = TRUE;
			}
			// we are closer to the next segment
			if (displacement > distance_to_next) 
				segment_navigated = TRUE;
			qDebug("First X[%.3f]Y[%.3f] Last=X[%.3f]Y[%.3f] Target Angle =[%.3f] Cur_Ang =[%.3f]", SegmentStart.x(),SegmentStart.y() ,SegmentEnd.x(),SegmentEnd.y() ,RTOD(angle),RTOD(EstimatedPos.phi));
			qDebug("Displ=[%.3f] Dist to Segend=[%.3f] D-Next=[%.3f]",displacement ,distance,distance_to_next);
			// Get the control Action to be applied
			cntrl = getAction(EstimatedPos.phi,angle,displacement,global_path->direction,velocity);
			// Angular Velocity Thrusholded
			if(cntrl.angular_velocity >   0.2)
				cntrl.angular_velocity =  0.2;
			if(cntrl.angular_velocity <  -0.2)
				cntrl.angular_velocity = -0.2;			
			if(log)
				fprintf(file,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %g %g\n",EstimatedPos.p.x(),EstimatedPos.p.y(),amcl_location.p.x(), amcl_location.p.y(), displacement ,error_orientation ,cntrl.angular_velocity,SegmentStart.x(),SegmentStart.y(),SegmentEnd.x(),SegmentEnd.y(),delta_timer.elapsed()/1e3,last_time);
			commManager->setSpeed(global_path->direction*cntrl.linear_velocity);
			commManager->setTurnRate(cntrl.angular_velocity);				
//			if(platform == WHEELCHAIR)
//				WCp->SetSpeed(global_path->direction*cntrl.linear_velocity,cntrl.angular_velocity);
//			else
//				pp->SetSpeed(global_path->direction*cntrl.linear_velocity,cntrl.angular_velocity);
//			g_timer_start(delta_timer);
		}
		qDebug("--->>>Finished Navigating this section, Moving to NEXT --->>>");
		first = last;
		if (!global_path->next)
		{
			qDebug("--->>> Destination Reached !!!");
			end_reached=TRUE;
			fflush(stdout);
		}
		else
			global_path = global_path->next;
	}
	commManager->setSpeed(0);
	commManager->setTurnRate(0);
//	if(platform == WHEELCHAIR)
//		WCp->SetSpeed(0,0); // Stop The motors
//	else
//		pp->SetSpeed(0,0);
	return;
}


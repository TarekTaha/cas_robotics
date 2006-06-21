#include "PathFollower.h"

namespace CasPlanner
{

PathFollower::PathFollower()
{
	// Do nothing, Empty Constructor
}
PathFollower::PathFollower(Node * p,double kd, double kt,double ko,double tracking_distance,GtkWidget * widget,bool log, PathPlanner *Planner,GdkPixbuf *pixbuf )
{
	this->path = p;
	this->kd = kd;
	this->kt = kt;
	this->ko = ko;
	this->tracking_distance = tracking_distance;
	this->widget = widget;
	this->speed = 0.1;
	this->Planner = Planner;
	this->pixbuf = pixbuf;
	robot = NULL; WCp = NULL; pp = NULL;  laser = NULL; localizer = NULL;
	avoidance_distance_left  = 0.2 + 0.5;
	avoidance_distance_right = 0.3;
	avoidance_distance_ahead = 0.2;
	stop = FALSE;
	this->log = log;
	if (log) 
		file = fopen("pathfollowing.txt","wb");
};
PathFollower::~PathFollower()
{
	if(platform == WHEELCHAIR)
		WCp->SetSpeed(0,0); // Stop The motors
	else
		pp->SetSpeed(0,0);
	if (robot)
		delete robot;
	if (WCp)
		delete WCp;
	if (pp)
		delete pp;
	if (laser)
		delete laser;
	if (localizer)
		delete localizer;
   	g_timer_destroy( timer2 );
   	g_timer_destroy( delta_timer );
	if(log)	
		fclose(file);
};
double PathFollower::Magnitude( Point p1, Point p2 )
{
    Point Vector;
    Vector.x = p2.x - p1.x;
    Vector.y = p2.y - p1.y;
    return sqrt( Vector.x * Vector.x + Vector.y * Vector.y);
}
double PathFollower::DistanceToLine( Point LineStart, Point LineEnd, Point P)
{
	double LineMag,distance;
	LineMag = Magnitude(LineStart,LineEnd);
	distance = (P.x*(LineStart.y - LineEnd.y) + P.y*(LineEnd.x - LineStart.x)+(LineStart.x*LineEnd.y - LineEnd.x*LineStart.y))/LineMag ;
    	return distance;
};
double PathFollower::DistToLineSegment(Point LineStart, Point LineEnd, Point p)
{
	Vector2D A(LineStart.x,LineStart.y),B(LineEnd.x,LineEnd.y),P(p.x,p.y);
  	//if the angle between PA and AB is obtuse then the closest vertex must be A
  	double dotA = (P.x - A.x)*(B.x - A.x) + (P.y - A.y)*(B.y - A.y);
  	if (dotA <= 0) 
		return Vec2DDistance(A, P);
	//if the angle between PB and AB is obtuse then the closest vertex must be B
  	double dotB = (P.x - B.x)*(A.x - B.x) + (P.y - B.y)*(A.y - B.y);
   	if (dotB <= 0) 
		return Vec2DDistance(B, P);
   	//calculate the point along AB that is the closest to P
  	//Vector2D Point = A + ((B - A) * dotA)/(dotA + dotB);
	//calculate the distance P-Point
  	//return Vec2DDistance(P,Point);
	return DistanceToLine(LineStart,LineEnd, p); 
};
void PathFollower::AddText ( char const * text)
	{
	GtkWidget * view;
	GtkTextBuffer *text_buffer;
	GtkTextIter     start, end;
	GtkTextMark* mark;
	//view = lookup_widget (GTK_WIDGET(widget),"textview1");
  	text_buffer = gtk_text_view_get_buffer (GTK_TEXT_VIEW (view));
	gtk_text_buffer_insert_at_cursor(text_buffer,text,-1);
	// Scrolling to end
	text_buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view));
    gtk_text_buffer_get_bounds (text_buffer, &start, &end);
   	mark = gtk_text_buffer_create_mark    (text_buffer, NULL, &end, 1);
   	gtk_text_view_scroll_to_mark(GTK_TEXT_VIEW(view), mark, 0.0, 0, 0.0, 1.0);
   	gtk_text_buffer_delete_mark (text_buffer, mark);
	}
void PathFollower::ResetWheelchair()
{
	if(platform == WHEELCHAIR && WCp)
	{
		/************** Wheelchair ON Automatic Mode **********************/
		WCp->SetPower(OFF);
		usleep(200000);
		WCp->SetPower(ON);
		usleep(200000);
		WCp->SetMode(AUTO);
		usleep(200000);
		AddText("\n	--->>> WheelChair is ON and CONTROL Mode is AUTO <<<---\n");
		fflush(stdout);
		/*****************************************************************/
	}	
}
void PathFollower::Connect(int platform)
{
	this->platform = platform;
	if(platform == WHEELCHAIR)
	{
		robot = new PlayerClient("192.168.0.101", 6665);
		WCp   = new WheelChairProxy(robot,0,'a');
		if(WCp->GetAccess() == 'e')
		{
			AddText("\n	--->>> Error getting wheelchair device atracking_distanceccess! <<<---");
			return;
		}		
	}
	else 
		robot = new PlayerClient("127.0.0.1", 6665);
	pp  		= new PositionProxy(robot,0,'a');
	laser 		= new LaserProxy(robot,0,'r');
	localizer 	= new LocalizeProxy(robot,0,'r');
 	if(localizer->access == 'e')
    {
  		AddText( "\n	--->>> Can't read from localizer <<<---" );
  		return;
   	} 
 	if(laser->access != 'r')
    {
      	AddText( "	--->>> Can't read from laser" );
      	return;
    }  
  	if(pp->GetAccess() == 'e') 
	{
    	AddText("\n	--->>> Error getting position device access! <<<---");
    	return;
  	}
	AddText(robot->conn.banner);
	ResetWheelchair();
}
void PathFollower::Localize()
{
	if(!path)
	{
		AddText("\n --->>> NO PATH to Localize<<<--- ");
		return;
	}
	pose[0]= path->location.x;
	pose[1]= path->location.y;
	pose[2]= path->angle;
	cout << "\n Default Pose given to the Localizer X="<<path->location.x<<" Y="<<path->location.y<<" Theta="<<path->angle;
	cout << "\n Tracking Distance="<<tracking_distance<<" Kd="<<kd<<" KTheta="<<kt;
	pose_var[0][0]=0.5;
	pose_var[0][1]=0.5;
	pose_var[0][2]=0.5;
	pose_var[1][0]=0.5;
	pose_var[1][1]=0.5;
	pose_var[1][2]=0.5;
	pose_var[2][0]=0.5;
	pose_var[2][1]=0.5;
	pose_var[2][2]=DTOR(10);
	localizer->SetPose(pose,pose_var);
	/***********************Finding Out where we are from the Localizer with initial estimation ************************/
	while(!position_found) //wait until we have 90% accurate assumption / hypothesis
	{
		// Block Until we get new data
		robot->Read();
		// Assumed number of Hypothesis
		printf("%d hypotheses\n", localizer->hypoth_count); 
    	printf("%d (weight %f): [ %f %f %f ]\n",0,localizer->hypoths[0].weight,localizer->hypoths[0].mean[0],localizer->hypoths[0].mean[1], localizer->hypoths[0].mean[2]);
		// Since the hypothesis are sorted, we assume the first one to be the one with the most weight
		if(localizer->hypoths[0].weight>=0.9) 
		{
			position_found=1; // Accurate Hypothesis found, so update current location and move next
			old_amcl.x = EstimatedPos.x=amcl_location.x= localizer->hypoths[0].mean[0];
			old_amcl.y = EstimatedPos.y=amcl_location.y= localizer->hypoths[0].mean[1];
			estimate_theta=localizer->hypoths[0].mean[2];
			localizer->SetPose(pose,pose_var);
		}
	}
	// Give me some time to setup
	usleep(10000000);
}
void PathFollower::RenderGui(Point amcl,double angle)
{
	GtkWidget * temp;
	Point p=amcl;
	Planner->ConvertToPixel(&p);
	//temp = lookup_widget (GTK_WIDGET(widget),"drawingarea1");
	p.x = p.x - 25;
	p.y = p.y - 25;
	if (p.x < 0) p.x = 0;
	if (p.y < 0) p.y = 0;
	if (p.x > Planner->map_width)  p.x = Planner->map_width;
	if (p.y > Planner->map_height) p.y = Planner->map_height;
  	gdk_draw_pixbuf(temp->window, NULL, pixbuf,
                    (int)p.x, (int)p.y,
                    (int)p.x, (int)p.y,
                    50, 50,
                    GDK_RGB_DITHER_NORMAL, 0, 0);
    Planner->draw_path();
	Planner->Translate_edges(amcl,angle);
	
	for (int i=0 ;i < 4; i++)
		Planner->ConvertToPixel(&Planner->translated_edge_points[i]);

	gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)Planner->translated_edge_points[0].x,(int)Planner->translated_edge_points[0].y,
	(int)Planner->translated_edge_points[1].x,(int)Planner->translated_edge_points[1].y);
	gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)Planner->translated_edge_points[1].x,(int)Planner->translated_edge_points[1].y,
	(int)Planner->translated_edge_points[2].x,(int)Planner->translated_edge_points[2].y);
	gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)Planner->translated_edge_points[2].x,(int)Planner->translated_edge_points[2].y,
	(int)Planner->translated_edge_points[3].x,(int)Planner->translated_edge_points[3].y);
	gdk_draw_line (temp->window,(GdkGC*)(temp)->style->white_gc,(int)Planner->translated_edge_points[3].x,(int)Planner->translated_edge_points[3].y,
	(int)Planner->translated_edge_points[0].x,(int)Planner->translated_edge_points[0].y);
	while (gtk_events_pending())
    	gtk_main_iteration();
}
ControlAction PathFollower::Controller(double angle_current,double angle_ref,double displacement,int direction)
{
	ControlAction cntrl;
	if(direction == -1)	  angle_current += M_PI;
	if(angle_ref     < 0) angle_ref    += 2*M_PI;
	if(angle_current < 0) angle_current+= 2*M_PI;
	double orientation_error = angle_current - angle_ref;
	if ( orientation_error >  M_PI) orientation_error= (-2*M_PI + orientation_error);
	if ( orientation_error < -M_PI) orientation_error= ( 2*M_PI + orientation_error);
	cntrl.angular_velocity = (-kd*speed*displacement - kt*speed*orientation_error);
	if (Abs(orientation_error)>DTOR(15))
	{
		cntrl.linear_velocity = 0;
		//cntrl.angular_velocity *=2;
	}
	else
		cntrl.linear_velocity = speed;
	AddText(g_strdup_printf("\n->Ori-Err=[%.3f] Dist-Err=[%.3f] Wdem=[%.3f]",RTOD(orientation_error),displacement,cntrl.angular_velocity));
	return cntrl;
};
void PathFollower::FollowPath(Node *path)
{
	ControlAction cntrl;
	if(!path)
	{
		AddText("\n --->>> No PATH TO FOLLOW <<<---");
		return ;
	}
	if(robot && pp && laser && localizer)
		Localize();
	else
	{
		AddText("\n No connected to Server , Reconnect and try again");
	}
	/********************** Start from the root and move forward ************************/
	first 		= path;
	prev_angle	= path->angle;
	path 		= path->next;
	/************************************************************************************/
	timer2 		= g_timer_new();
	delta_timer = g_timer_new();
	//Reset Times
	last_time=0; delta_t=0;	velocity=0;
	while(!end_reached) 
	{
		g_timer_start(timer2);
		g_timer_start(delta_timer);
		robot->Read();
		last = path;
		ni = first->location; SegmentStart.x = ni.x; SegmentStart.y = ni.y;
		AddText(g_strdup_printf("\n	--->>>NEW Line SEG Starts x[%.3f]y[%.3f]",SegmentStart.x,SegmentStart.y));
		fflush(stdout);
		ni = last->location;  SegmentEnd.x = ni.x;  SegmentEnd.y = ni.y;	
		AddText(g_strdup_printf(" Ends at   x[%.3f]y[%.3f] <<<---",SegmentEnd.x,SegmentEnd.y));
		direction = -1;
		angle = atan2(SegmentEnd.y - SegmentStart.y,SegmentEnd.x - SegmentStart.x);
		AddText(g_strdup_printf("\n	--->>> Orientation(Planned) to follow :=%.3f <<<---",RTOD(angle)));
		segment_navigated = FALSE;
		while (!segment_navigated && !stop) // Loop for each path segment
		{
			obstacle_avoidance_force = 0;
			RenderGui(EstimatedPos,estimate_theta);
			
			// Estimate the location based on the Last AMCL location and the vehichle model
			delta_t=g_timer_elapsed(delta_timer, NULL );
			estimate_theta += pp->SideSpeed()*delta_t;
			EstimatedPos.x += velocity*cos(estimate_theta)*delta_t;
			EstimatedPos.y += velocity*sin(estimate_theta)*delta_t;
			//cout<<"\nVelocity is:"<<velocity<<" Side Speed is:"<<pp->SideSpeed();
			// Check if new data arrived
			//if(robot->Peek(0))
			robot->Read();
			{
				//if(!robot->Read()) // 				No errors occured while reading
				{
					if (velocity!= pp->Speed()) //	Velocity Changed?
						velocity = pp->Speed();
					//cout<<"\n New data arrived Velocity="<<pp->Speed()<<" Angular"<<pp->SideSpeed();
					for(int i=0;i<localizer->hypoth_count;i++)
					{
						// Do we have an accurate Hypothesis?
						if(localizer->hypoths[i].weight>=0.8)
						{
							amcl_location.x = localizer->hypoths[i].mean[0];
							amcl_location.y = localizer->hypoths[i].mean[1];
							theta = localizer->hypoths[i].mean[2];
							// Is it a new hypothesis (not the same as the last)?
							if(old_amcl.x !=amcl_location.x || old_amcl.y !=amcl_location.y )
							{
								// Recording the last time Data changed
								last_time = g_timer_elapsed(timer2, NULL );
								// resetting the timer
								g_timer_start(timer2); 
								// Override the Estimated Location with the AMCL hypothesis
								old_amcl.x = EstimatedPos.x = amcl_location.x;
								old_amcl.y = EstimatedPos.y = amcl_location.y;
								estimate_theta = theta;
							}
						}
					}
				}
			}
			// Determine the Tracking Point 
			AddText(g_strdup_printf("\n->Vel =%.3f m/sev X=[%.3f] Y=[%.3f] Theta=[%.3f] time=%g",pp->Speed(),EstimatedPos.x,EstimatedPos.y,RTOD(estimate_theta),delta_t));
			tracking_point.x = EstimatedPos.x + tracking_distance*cos(estimate_theta) - 0*sin(estimate_theta);
			tracking_point.y = EstimatedPos.y + tracking_distance*sin(estimate_theta) + 0*cos(estimate_theta); 
			// Distance to the path Segment
			distance = sqrt(pow(SegmentEnd.x-tracking_point.x,2)+pow(SegmentEnd.y-tracking_point.y,2));
			displacement = DistToLineSegment(SegmentStart,SegmentEnd,tracking_point);
			// Did we reach the last segment ???
			if (path->next) // NO we didnt
			{
				Point n;
				n.x = path->next->location.x;
				n.y = path->next->location.y;
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
			AddText(g_strdup_printf("\n->First X[%.3f]Y[%.3f] Last=X[%.3f]Y[%.3f] Target Angle =[%.3f] Cur_Ang =[%.3f]", SegmentStart.x,SegmentStart.y ,SegmentEnd.x,SegmentEnd.y ,RTOD(angle),RTOD(estimate_theta)));
			AddText(g_strdup_printf("\n->Displ=[%.3f] Dist to Segend=[%.3f] D-Next=[%.3f]",displacement ,distance,distance_to_next));
			// Get the control Action to be applied
			cntrl = Controller(estimate_theta,angle,displacement,path->direction);
			// Angular Velocity Thrusholded
			if(cntrl.angular_velocity >   0.2)
				cntrl.angular_velocity =  0.2;
			if(cntrl.angular_velocity <  -0.2)
				cntrl.angular_velocity = -0.2;			
			if(log)
				fprintf(file,"%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %g %g\n",EstimatedPos.x,EstimatedPos.y,amcl_location.x, amcl_location.y, displacement ,error_orientation ,cntrl.angular_velocity,SegmentStart.x,SegmentStart.y,SegmentEnd.x,SegmentEnd.y,g_timer_elapsed(delta_timer, NULL ),last_time);
			if(platform == WHEELCHAIR)
				WCp->SetSpeed(path->direction*cntrl.linear_velocity,cntrl.angular_velocity);
			else
				pp->SetSpeed(path->direction*cntrl.linear_velocity,cntrl.angular_velocity);
			g_timer_start(delta_timer);
		}
		AddText("\n--->>>Finished Navigating this section, Moving to NEXT --->>>\n");
		first = last;
		if (!path->next)
		{
			AddText("\n--->>> Destination Reached !!!");
			end_reached=TRUE;
			fflush(stdout);
		}
		else
			path = path->next;
	}
	if(platform == WHEELCHAIR)
		WCp->SetSpeed(0,0); // Stop The motors
	else
		pp->SetSpeed(0,0);
}
}

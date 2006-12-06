#include "forcefield.h"

ForceField::ForceField(Robot r,ConfigFile *cf):
Robot(r),
INF(1E200),
EP(1E-10)	  
{
	int numSec; 
	numSec = cf->GetSectionCount(); 
	for(int i=0; i < numSec; i++)
	{
	    QString sectionName = cf->GetSectionType(i),FF;
	    if(sectionName == "ForceField")
	    {
	    	FF		   =   cf->ReadString(i,"FF","VariableSpeedFF");
	    	if(FF == "VariableSpeedFF")
	    	{
	    		FF_algorithm = VariableSpeedFF;
	    	}
	    	else
	    	{
	    		FF_algorithm = SimpleFF;
	    	}   	
		 	FixedRatio =   cf->ReadFloat (i, "FixedRatio", 0.2);
		  	TimeStep =     cf->ReadFloat (i, "TimeStep", 0.2);
		  	SysK   =       cf->ReadFloat (i, "SysK", 5);
		  	SysC =         cf->ReadFloat (i, "SysC", 1.25);
			SysP    = 	   cf->ReadFloat (i, "SysP", 10);
			SysQ   = 	   cf->ReadFloat (i, "SysQ",10);
			MaxSpeed   =   cf->ReadFloat (i, "MaxSpeed", 0.2);
			MaxAcceT     = cf->ReadFloat (i, "MaxAcceT", 0.2);
			OmegadotMax  = cf->ReadFloat (i, "OmegadotMax", 0.15);
			OmegaMax     = cf->ReadFloat (i, "OmegaMax", 0.2);
			curvefittingorder= cf->ReadInt (i, "curvefittingorder",6);
			Gapdist          = cf->ReadFloat (i, "Gapdist", 0.5);
			NPOL             = cf->ReadFloat (i, "NPOL", 4);									
		}
	}
}
// Destructor -> Free Memory
ForceField::~ForceField()
{
}
double ForceField::Dist2Robot(QPointF ray_end,double &angle)
{
	QPointF temp[4],intersection;
    double LineMag, U;
    QPointF Intersection;
	Line L1,L2;
	double dist,shortest_dist=1000;
	for(int i=0;i<local_edge_points.size();i++)
	{
		temp[i] = local_edge_points[i];
		temp[i] = Trans2Global(temp[i],robotLocation);
	}
//	ray_end = Trans2Global(ray_end,laser_pose);
	for(int j=0;j<4;j++)
	{
		L1.SetStart(temp[j%4]);      L1.SetEnd(temp[(j+1)%4]);
	    LineMag = L1.LineMag();
	    U = ((( ray_end.x() - L1.start.x() ) * ( L1.end.x() - L1.start.x() )) +
	        ((  ray_end.y() - L1.start.y() ) * ( L1.end.y() - L1.start.y() )))
	        /( LineMag * LineMag );
		if( U < 0.0f || U > 1.0f) 
		{
			if (Dist(L1.start,ray_end) < Dist(L1.end,ray_end))
			{
			    Intersection.setX(L1.start.x());
			    Intersection.setY(L1.start.y());					
			    dist = Dist(L1.start,ray_end);
			}
			else
			{
			    Intersection.setX(L1.end.x());
			    Intersection.setY(L1.end.y());					
			    dist = Dist(L1.end,ray_end);			
			}			
		}
		else
		{
		    Intersection.setX(L1.start.x() + U * ( L1.end.x() - L1.start.x() ));
		    Intersection.setY(L1.start.y() + U * ( L1.end.y() - L1.start.y() ));
		    dist = Magnitude( ray_end, Intersection );		
		}
		if(dist < shortest_dist)
		{
			shortest_dist = dist;
			this->intersect_point = intersection;
			this->end_point = ray_end;
		}
	}
	angle = ATAN2(ray_end,Intersection);
	return shortest_dist;
};
void ForceField::CrossProduct(double MatrixA[3], double MatrixB[3], double MatrixC[3])
{
 	MatrixC[0] = (MatrixA[1] * MatrixB[2]) - (MatrixB[1] * MatrixA[2]);
	MatrixC[1] = (MatrixA[2] * MatrixB[0]) - (MatrixB[2] * MatrixA[0]);
 	MatrixC[2] = (MatrixA[0] * MatrixB[1]) - (MatrixB[0] * MatrixA[1]);
};	
velVector ForceField::GenerateField(Pose pose,LaserScan laser_set,Pose Goal,double speed,double turnrate,QVector <Robot*> robots,double deltaTime)
{
	QVector<Interaction> robots_interaction_set;
 	double coefficient[curvefittingorder];
	double sum_x, average_x, inter_dmax, inter_dmin, DiffInterPoint, Xoffset, NormInterPoint, closestdist_ob, closestdist_ob_angle; 
	QVector< QVector<QPointF> > obstacles_set = DivObst(laser_set,Pose(0,0,0));
	QVector<Interaction> obstacle_interaction_set;
	Interaction inter_point, max_inter;	
		
	//qDebug("here"); fflush(stdout);
	velVector action;
 	this->robotLocation = pose;
 	//qDebug("Robot Pose x:%f y:%f phi%f",robotLocation.p.x(),robotLocation.p.y(),robotLocation.phi);
 	this->goalLocation  = Goal;
 	this->robotSpeed = speed;
 	this->robotTurnRate = turnrate;	
 	//qDebug ("before Turnrate=%f", robotTurnRate); 
 	//qDebug ("Direc=%f, Turnate=%f", robotLocation.phi,robotTurnRate);  
 	
 	/*************Static Obstacles*********************/ 

	//qDebug ("obstacles_set.size()=%d",obstacles_set.size());fflush(stdout); 
	
	for (int i = 0; i <obstacles_set.size() ; i++)
 	{
 		max_inter.force = -100;
 		sum_x = 0;
 		average_x = 0;
 		inter_dmax = 0;
 		inter_dmin = 0;
 		closestdist_ob = 0;
 		closestdist_ob_angle = 0;
 		int j;
 		for (j =0; j < obstacles_set[i].size();j++)
 		{
 			inter_point.location.setX(obstacles_set[i][j].x());
 			inter_point.location.setY(obstacles_set[i][j].y());	
 			//qDebug ("X=%f, Y=%f", inter_point.location.x(), inter_point.location.y()); 			
 			inter_point.force = ForceValue(inter_point.location, inter_dmax, inter_dmin, closestdist_ob, closestdist_ob_angle);
 			inter_point.closestdist = closestdist_ob;
 			inter_point.angle = closestdist_ob_angle;
 			//inter_point.force = ForceValue(inter_point.location);
 			sum_x += inter_point.location.x();
 			if(inter_point.force > max_inter.force )
 			{
 				max_inter = inter_point;
 				inter_point.direction = 0;
 				
 				//qDebug ("i=%d, Dmax=%f, Dmin=%f, force=%f", i, inter_dmax, inter_dmin, inter_point.force);
 			}
 			//qDebug ("i=%d, X=%f, Y=%f, force=%f", i, inter_point.location.x(), inter_point.location.y(), inter_point.force);
 		}
 		average_x = sum_x/(j+1);		
	   	if (!IsZero(max_inter.force))
	   	{
	   		//qDebug("MAX INTER FORCE:%f",max_inter.force); 	   		
     		LSCurveFitting(obstacles_set[i],coefficient,curvefittingorder);
     		DiffInterPoint = coefficient[1], Xoffset = max_inter.location.x() - average_x;
     		for (int diffo = 2; diffo < curvefittingorder; diffo++)
     		{
       			DiffInterPoint = diffo * coefficient[diffo] * pow(Xoffset, diffo - 1) + DiffInterPoint;
     		}
     		NormInterPoint = FindNorm(max_inter.location,DiffInterPoint);
     		max_inter.direction = NormInterPoint;
			obstacle_interaction_set.push_back(max_inter);
			//qDebug("MAX INTER FORCE:%f, direction: %f, x: %f, y: %f",max_inter.force, max_inter.direction, max_inter.location.x(), max_inter.location.y());
	   	}
	}
	/*************Static Obstacles*********************/
	
	/*************Dynamic Obstacles*********************/
	robots_interaction_set = getDynamicInteractionSet(robots);
	/*************Dynamic Obstacles*********************/
		
	if(FF_algorithm == VariableSpeedFF)
	{
		//qDebug("Obstacle Interaction Set Before VSFF=%d",obstacle_interaction_set.size());
 		VSFF(obstacle_interaction_set, robots_interaction_set, deltaTime);
	}
	else
	{
		//qDebug("Before SimFF"); fflush(stdout);		
		SimFF(obstacle_interaction_set, robots_interaction_set, deltaTime);
		//qDebug("After SimFF"); fflush(stdout);				
	}
 	action.speed= robotSpeed;
 	action.turnRate = robotTurnRate;
	//qDebug("Function Ended"); fflush(stdout);
 	return action;
};
QVector<Interaction>  ForceField::getDynamicInteractionSet(QVector <Robot*> robots)
{
	QVector<Interaction> robots_interaction_set;
	double coefficient_robots[curvefittingorder],inter_robot_dmax,
		   inter_robot_dmin,sum_x_robots,average_x_robots,DiffInterPoint_robots,
		   NormInterPoint_robots, Xoffset_robots, closestdist_ro, closestdist_ro_angle;
	int DmaxCenter=0;
	Interaction max_inter_robots, inter_point_robots;
	QVector<QPointF> Dmax, Dmin;
	QVector<QPointF> robotsDmax_set;
	for (int i_robots = 0; i_robots < robots.size() ; i_robots ++)
	{
		//qDebug("robot %s:  size=%f,Speed=%f, Dirc=%f, X=%f, Y=%f, ",qPrintable(robots[i_robots]->robotName),robots[i_robots]->robotRadius, robots[i_robots]->robotSpeed,robots[i_robots]->robotLocation.phi, robots[i_robots]->robotLocation.p.x(), robots[i_robots]->robotLocation.p.y());
		//qDebug("1i = %d", i_robots);fflush(stdout);
		//qDebug("HERE START"); fflush(stdout);
		
		//qDebug("HERE 1"); fflush(stdout);
		robotForceFieldShape(robots[i_robots], Dmax, Dmin);
		//qDebug("HERE 2"); fflush(stdout);
		
		max_inter_robots.force = 0;
		max_inter_robots.direction = 0;
		inter_point_robots.direction = 0;
		//qDebug("HERE 3"); fflush(stdout);
		//DmaxCenter = 0;
		//qDebug("HERE 4"); fflush(stdout);
		inter_robot_dmax = 0;
		inter_robot_dmin = 0;
		closestdist_ro = 0;
		closestdist_ro_angle = 0;
		for (int indx = 0; indx < 360; indx++)
		{
			inter_point_robots.location.setX(Dmax[indx].x());
			inter_point_robots.location.setY(Dmax[indx].y());
			inter_point_robots.force = ForceValue(inter_point_robots.location, inter_robot_dmax, inter_robot_dmin, closestdist_ro, closestdist_ro_angle);
			inter_point_robots.closestdist = closestdist_ro;
			inter_point_robots.angle = closestdist_ro_angle;
			//qDebug ("inter_point_robots.force=%f, max_inter_robots.force=%f", inter_point_robots.force, max_inter_robots.force);
			//qDebug("HERE 5"); fflush(stdout);
			if (inter_point_robots.force > max_inter_robots.force)
			{
				max_inter_robots = inter_point_robots;
				//qDebug("force=%f, direc=%f, x=%f, y=%f", max_inter_robots.force,max_inter_robots.direction,max_inter_robots.location.x(),max_inter_robots.location.y());
				//inter_point_robots.direction = 0;
				DmaxCenter = indx;
				//qDebug("indx=%d,DmaxCenter=%d", indx, DmaxCenter);fflush(stdout);
				//qDebug ("robotDmax=%f, robotDmin=%f, robotforce=%f", inter_robot_dmax, inter_robot_dmin, inter_point_robots.force);
				//qDebug("HERE 6"); fflush(stdout);
			}
		}
				
		sum_x_robots = 0;
		if (!IsZero(max_inter_robots.force))
		{
			QPointF temp;
			for (int i = 0; i < 20; i++)
			{
				//qDebug("DmaxCenter=%d", DmaxCenter);fflush(stdout);
				if (i <= (10 - DmaxCenter))
				{
					temp.setX(Dmax[DmaxCenter - 10 + i + 360].x());
					temp.setY(Dmax[DmaxCenter - 10 + i + 360].y());
					robotsDmax_set.push_back(temp);
				}
				else
				{
					temp.setX(Dmax[DmaxCenter - 10 + i].x()); //how about if i < 10
					temp.setY(Dmax[DmaxCenter - 10 + i].y());
					robotsDmax_set.push_back(temp);
				}
				sum_x_robots = sum_x_robots + robotsDmax_set[i].x();
			}
			//qDebug("HERE 7"); fflush(stdout);
			average_x_robots = sum_x_robots/(20); 	   		
	 		LSCurveFitting(robotsDmax_set,coefficient_robots,curvefittingorder);
	 		DiffInterPoint_robots = coefficient_robots[1], Xoffset_robots = max_inter_robots.location.x() - average_x_robots;
	 		//qDebug("HERE 8"); fflush(stdout);
	 		for (int diffo_new = 2; diffo_new < curvefittingorder; diffo_new ++)
	 		{
	   			DiffInterPoint_robots = diffo_new * coefficient_robots[diffo_new] * pow(Xoffset_robots, diffo_new - 1) + DiffInterPoint_robots;
	 		}
	 		//qDebug("HERE 9"); fflush(stdout);
	 		NormInterPoint_robots = FindNorm(max_inter_robots.location,DiffInterPoint_robots);
	 		//qDebug("HERE 91"); fflush(stdout);
	 		max_inter_robots.direction = NormInterPoint_robots;
	 		//qDebug("robotsinter_size=%d", robots_interaction_set.size());
	 		//qDebug("HERE 92"); fflush(stdout);
			robots_interaction_set.push_back(max_inter_robots);
			//qDebug("force=%f, direc=%f, x=%f, y=%f", max_inter_robots.force,max_inter_robots.direction,max_inter_robots.location.x(),max_inter_robots.location.y());
			//qDebug("robotsinter_size=%d", robots_interaction_set.size());
			//qDebug("HERE 10"); fflush(stdout);
		}
		//qDebug("2i = %d", i_robots);fflush(stdout);
		//qDebug("HERE END"); fflush(stdout);
		//qDebug("3i = %d", i_robots); fflush(stdout);
	}
	//qDebug("InteractionSet Size=%d",robots_interaction_set.size()); fflush(stdout);
	return robots_interaction_set;
	/*************Dynamic Obstacles*********************/ 	
}
double ForceField::FindNorm(QPointF interaction_point, double Tang)
{
	double Direction = 0;
	double k, x1, x2, y1, y2, Value, Value1, Value2;
	if (!IsZero(Tang))
 	{
   		k = -1 / Tang;
    	x1 = interaction_point.x() + 1;
    	x2 = interaction_point.x() - 1;
    	y1 = interaction_point.y() + k;
    	y2 = interaction_point.y() - k;
    	Value = robotLocation.p.y() - interaction_point.y() - Tang * (robotLocation.p.x() - interaction_point.x());
    	Value1 = y1 - interaction_point.y() - Tang * (x1 - interaction_point.x());
    	Value2 = y2 - interaction_point.y() - Tang * (x2 - interaction_point.x());
     	if (IsZero(Value))
     		{
       			Direction = atan2(robotLocation.p.y() - interaction_point.y(), robotLocation.p.x() - interaction_point.x());
     		}
      	else if ((Value * Value1) > 0)
      		{
      			Direction = atan2(k, 1);
      		}
      	else if ((Value * Value2) > 0)
      		{
       			Direction = atan2(-k, -1);
      		}
      	else
      		{
        		Direction = 0;
        		//qDebug ("error!");
      		}
    	}
    	else
    	{
      		if (robotLocation.p.y() > (interaction_point.y() + EP))
      		{
        		Direction = M_PI / 2;
      		}
      		else if (robotLocation.p.y() < (interaction_point.y() - EP))
      		{
       		 	Direction = -M_PI / 2;
      		}
      		else if (robotLocation.p.x() < interaction_point.x())
      		{
       			Direction = M_PI;
      		}
      		else
      		{
        		Direction = 0; //may have some problems
      		}
   	}
	return (Direction);
};
//double ForceField::FindNorm(QPointF interaction_point, double Tang)
//{
//	//DotMultiply(QPointF p1,QPointF p2,QPointF p0)
//	double NormDir, Norm;
//	if (IsZero(Tang))
//	{
//		Norm = INF;
//	}
//	else
//	{
//		Norm = 1 / Tang;
//	}
//	QPointF p1, p2, pR;
//	pR.setX(robotLocation.p.x()); pR.setY(robotLocation.p.y());
//	p1.setX(interaction_point.x() + 1); p1.setY(interaction_point.y() + Norm);
//	p2.setX(interaction_point.x() - 1); p2.setY(interaction_point.y() - Norm);
//	qDebug ("Tang=%f, p1X=%f, p1Y=%f, p2X=%f, p2Y=%f, pRX=%f, pRY=%f", Tang, p1.x(), p1.y(), p2.x(), p2.y(), pR.x(), pR.y());
//	double Value1 = DotMultiply(p1, pR, interaction_point);
//	double Value2 = DotMultiply(p2, pR, interaction_point);
//	if (Value1 > 0)
//	{
//		NormDir = ATAN2(p1, interaction_point);
//	}
//	else if (Value1 < 0)
//	{
//		NormDir = ATAN2(p2, interaction_point);
//	}
//	elseVSFF
//	{
//		NormDir = M_PI / 2;
//	}
//	qDebug ("Value1=%f, Value2=%f, NormDir=%f", Value1, Value2, NormDir);
//	return(NormDir);
//}

double ForceField::ForceValue(QPointF ray_end, double &DMAX, double &DMIN, double &closest_dist, double &angle )
{
// 	double MinSpeed = 0.01, Speed = robotSpeed;
// 	if (robotSpeed < MinSpeed)
// 	{
// 		Speed = MinSpeed;
//	}
// 	if ((robotSpeed > - MinSpeed) && (robotSpeed < 0))
// 	{
// 		robotSpeed = - MinSpeed;
// 	}
	//double angle,closest_dist;
	//double angle;
	closest_dist = Dist2Robot(ray_end,angle);
 	double Er = robotSpeed / (MaxSpeed * SysC);
 	double Safedist = 0.1;
 	DMAX = MAX (SysK * Er * robotRadius / (1 - Er * cos(angle)), Safedist / FixedRatio);
 	DMIN = FixedRatio * DMAX;
 	//qDebug ("Dmin=%f, Dmax=%f, clost_dist=%f", DMIN, DMAX, closest_dist);
 	double Ratio = closest_dist / DMAX;
 	double ForceAmp;
 	if (Ratio >= 1)
 	{
   		ForceAmp = 0.0;
 	}
 	else if (Ratio >= FixedRatio)
 	{
   		ForceAmp = SysP * (1 - Ratio) / (1 - FixedRatio);
 	}
 	else
 	{
   		ForceAmp = (1 - Ratio / (1.1 * FixedRatio)) * SysP * 33;
 	}
 	//qDebug ("Ratio=%f,ForceAmp=%f, closest_dist=%f, DMAX=%f",Ratio,ForceAmp, closest_dist, DMAX);
	//qDebug("Er:%f Angle:%f Dmax:%f Dmin:%f Ratio:%f ClosestDist:%f ForceAmp:%f",Er,angle,Dmax,Dmin,Ratio,closest_dist,ForceAmp); 	
 	return(ForceAmp);
}

QVector < QVector<QPointF> > ForceField::DivObst(LaserScan laser_set,Pose laser_pose)
{
 	double DistBetw;
 	QPointF p1,p2;
	QVector < QVector<QPointF> > obstacles_set;
	QVector<QPointF> obstacle;
	p1.setX(laser_set.points[1].x()); p1.setY(laser_set.points[1].y());
	//qDebug("first P1X=%f, P1Y=%f", p1.x(), p1.y());		double 
	obstacle.push_back(p1);		
	for (int i = 1; i < laser_set.points.size() - 1; i++)
 	{
		p1.setX(laser_set.points[i].x()); p1.setY(laser_set.points[i].y());
		p1 = Trans2Global(p1,laser_pose);p1 = Trans2Global(p1,robotLocation);
		p2.setX(laser_set.points[i+1].x())	; p2.setY(laser_set.points[i+1].y());	
		p2 = Trans2Global(p2,laser_pose);p2 = Trans2Global(p2,robotLocation);				
   		DistBetw = Dist(p1,p2);
   		if (DistBetw < Gapdist)
   		{
     		obstacle.push_back(p1);	
     		if (i == laser_set.points.size() - 2)
     		{
     			obstacle.push_back(p2);
     			obstacles_set.push_back(obstacle);
     		}
     		//qDebug("P1X=%f, P1Y=%f, i=%d, size=%d, obstacles=%d", p1.x(), p1.y(), i, laser_set.points.size(), obstacles_set.size());
   		}
   		else
   		{
   			obstacles_set.push_back(obstacle);
   			obstacle.clear();
   			obstacle.push_back(p2);
   			//qDebug("P2X=%f, P2Y=%f", p2.x(), p2.y());
   		}
	}
	return obstacles_set;
}

/*****************************************************************/
// use least square method in curve fitting
// x[n]  x values of n data
// y[n]  y values of n data
// n number of data
// a[m] coefficient of x*
// dt1: sum of square errors; dt2: sum of absolute error; dt3: max absolute error
// for m = 3, P(x) = a[0] + a[1] * (x - Xave) + a[2] * (x - Xave) ^ 2
// highest degree is 20. m <= n and m <= 20.
// if m > n or m > 20, then m = min {n, 20}
void ForceField::LSCurveFitting (QVector<QPointF> obstacle, double a[], int m)
{
	int i, j, k;
	int n = obstacle.size();
 	double z, p, c, g, q=0, d1, d2, s[20], t[20], b[20];
 	for (i = 0; i <= (m - 1); i++)
   		a[i] = 0.0;
 	if (m > n)
   		m = n;
 	if (m > 20)
   		m = 20;
 	z = 0.0;
 	for (i = 0; i <= (n - 1); i++)
   		z = z + obstacle[i].x()/(1.0 * n);
 	b[0] = 1.0;
 	d1 = 1.0 * n;
 	p = 0.0;
 	c = 0.0;
 	for (i = 0; i <= (n - 1); i++)
 	{
   		p = p + (obstacle[i].x() - z);
   		c = c + obstacle[i].y();
 	}
 	c = c / d1;
 	p = p / d1;
 	a[0] = c * b[0];
 	if (m > 1)
 	{
   		t[1] = 1.0;
   		t[0] = -p;
   		d2 = 0.0;
   		c = 0.0;
   		g = 0.0;
   		for (i = 0; i <= (n - 1); i++)
   		{
     		q = obstacle[i].x() - z - p;
     		d2 = d2 + q * q;
     		c = c + obstacle[i].y() * q;
     		g = g + (obstacle[i].x() - z) * q * q;
   		}
   		c = c / d2;
   		p = g / d2;
   		q = d2 / d1;
   		d1 = d2;
   		a[1] = c * t[1];
   		a[0] = c * t[0] + a[0];
 	}
 	for (j = 2; j <= (m - 1); j++)
 	{
   		s[j] = t[j - 1];
   		s[j - 1] = -p * t[j - 1] + t[j - 2];
   		if (j >= 3)
     	for (k = j - 2; k >= 1; k--)	
       		s[k] = -p * t[k] + t[k - 1] - q * b[k];
   		s[0] = -p * t[0] - q * b[0];
   		d2 = 0.0;
   		c = 0.0;
   		g = 0.0;
   		for (i = 0; i <= (n - 1); i++)
   		{
     		q = s[j];
     		for (k = j - 1; k >= 0; k--)
       			q = q * (obstacle[i].x() - z) + s[k];
     		d2 = d2 + q * q;
     		c = c + obstacle[i].y() * q;
     		g = g + (obstacle[i].x() - z) * q * q;
   		}
   		c = c / d2;
   		p = g / d2;
   		q = d2 / d1;
   		d1 = d2;
   		a[j] = c * s[j];
   		t[j] = s[j];
   		for (k = j - 1; k >= 0; k--)
   		{
     		a[k] = c * s[k] + a[k];
     		b[k] = t[k];
     		t[k] = s[k];
   		}
 	}
 	return;
}

/*****************************************************************/
void ForceField::SimFF(QVector<Interaction> obstacle_interaction_set, QVector<Interaction> robots_interaction_set, double realtime)
{
	
	//qDebug ("Simple FF!"); fflush(stdout);
	realtime = 0 ;
	qDebug ("OldSpeed=%f, OldDirec=%f, Turnate=%f", robotSpeed,robotLocation.phi,robotTurnRate);  
	//double FattAmp = SysQ;
  	double FattAngleA[2] = {goalLocation.p.x() - robotLocation.p.x(), goalLocation.p.y() - robotLocation.p.y()};
  	double FattAngle = atan2(FattAngleA[1], FattAngleA[0]);
  	//double FattX = FattAmp * cos(FattAngle);
  	//double FattY = FattAmp * sin(FattAngle);
  	//qDebug ("Simple FF 1!"); fflush(stdout);
  	//qDebug ("FattAmp = %f, FattAngle = %f, FattX = %f, FattY = %f, FattAngleA[0] = %f, FattAngleA[1] = %f",FattAmp, FattAngle, FattX, FattY, FattAngleA[0], FattAngleA[1]);
  	double FrepXTotal = 0, FrepYTotal = 0, FrepXTotal_robots = 0, FrepYTotal_robots = 0, anglebetw, MaxRep = 0;
  	double Mindist = 10, Mindist_angle = 0, FrepAmp, FrepAngle, FrepX, FrepY, FrepAmp_robots, FrepAngle_robots, FrepX_robots,FrepY_robots;
  	for(int i = 0; i < obstacle_interaction_set.size();i++)
  	{
  		//qDebug ("i=%d, force=%f, direction=%f", i, obstacle_interaction_set[i].force, obstacle_interaction_set[i].direction);
    	FrepAmp   = obstacle_interaction_set[i].force;
    	FrepAngle = obstacle_interaction_set[i].direction;
    	FrepX = FrepAmp * cos(FrepAngle);
    	FrepY = FrepAmp * sin(FrepAngle);
    	qDebug ("i=%d, ObstacleX=%f, ObstacleY=%f, FrepAmp = %f, FrepAngle = %f, FrepX = %f, FrepY = %f", i, obstacle_interaction_set[i].location.x(), obstacle_interaction_set[i].location.y(), FrepAmp, FrepAngle, FrepX, FrepY);
    	FrepXTotal = FrepXTotal + FrepX;
    	FrepYTotal = FrepYTotal + FrepY;
    	if (FrepAmp > MaxRep)
    	{
    		MaxRep = FrepAmp;
    		Mindist = obstacle_interaction_set[i].closestdist;
    		Mindist_angle = obstacle_interaction_set[i].angle;
    	}
    }
  	///qDebug ("Simple FF 2!"); fflush(stdout);
  	for(int i = 0; i < robots_interaction_set.size();i++)
  	{
    	FrepAmp_robots = robots_interaction_set[i].force;
    	FrepAngle_robots = robots_interaction_set[i].direction;
    	FrepX_robots = FrepAmp_robots * cos(FrepAngle_robots);
    	FrepY_robots = FrepAmp_robots * sin(FrepAngle_robots);
    	qDebug ("i=%d, robotinterX=%f, robotinerY=%f, FrepAmp = %f, FrepAngle = %f, FrepX = %f, FrepY = %f", i, robots_interaction_set[i].location.x(), robots_interaction_set[i].location.y(), FrepAmp_robots, FrepAngle_robots, FrepX_robots, FrepY_robots);
    	FrepXTotal_robots = FrepXTotal_robots + FrepX_robots;
    	FrepYTotal_robots = FrepYTotal_robots + FrepY_robots;
    	if (FrepAmp_robots > MaxRep)
    	{
    		MaxRep = FrepAmp_robots;
    		Mindist = robots_interaction_set[i].closestdist;
    	}
    }
    //qDebug ("Simple FF 3!"); fflush(stdout);
    
//    double FtotalX = FattX + FrepXTotal + FrepXTotal_robots;
//  	double FtotalY = FattY + FrepYTotal + FrepYTotal_robots;
    double FreptotalX = FrepXTotal + FrepXTotal_robots;
  	double FreptotalY = FrepYTotal + FrepYTotal_robots;
  	//double Frepangle = atan2(FreptotalX, FreptotalY)
  	double Frepmag = sqrt(FreptotalX * FreptotalX + FreptotalY * FreptotalY );
  	//double Frepangle = atan2(FreptotalY, FreptotalX);
  	
  	double factor;
  	if (Frepmag > 0.001)
  	{
  	//factor=1.5 * Frepmag;
  	factor = SysQ;
  	}
  	else
  	{
  	factor = SysQ;
  	}
  	
  	double FattX = factor * cos(FattAngle);
    double FattY = factor * sin(FattAngle);
    double FtotalX = FattX + FreptotalX;
  	double FtotalY = FattY + FreptotalY;
  	
  	double ForceAngle = atan2(FtotalY, FtotalX);
//     qDebug("Frepmag=%f, Frepangle=%f, FattAngle=%f, ForceAngle=%f", Frepmag, Frepangle, FattAngle, ForceAngle);
    qDebug("FattX=%f, FattY=%f, FreptotalX=%f, FreptotalY=%f, FtotalX=%f, FtotalY=%f", FattX, FattY, FreptotalX, FreptotalY, FtotalX, FtotalY);  	
  	double robotSpeed_new = MAX(MaxSpeed / (1 + MaxRep / SysP), 0.05);
  	double MaxSpeedIncr = 0.01;
  	if ((robotSpeed_new - robotSpeed) > MaxSpeedIncr)
  	{
  		robotSpeed = robotSpeed + MaxSpeedIncr;
  	}
  	else if ((robotSpeed_new - robotSpeed) < -MaxSpeedIncr)
  	{
  		robotSpeed = robotSpeed - MaxSpeedIncr;
  	}
  	else
  	{
  		robotSpeed = robotSpeed_new;
  	}

  	anglebetw = Delta_Angle (robotLocation.phi, ForceAngle);
  	robotTurnRate = DTOR(40)*((anglebetw)/M_PI);
	//qDebug ("\tRobotTurnRate_desired=%f, Max Velocity=%f, turnRate_incr_chosen=%f", robotTurnRate_desired, velocityMax, turnRate_incr_chosen);  
  	qDebug ("\tOUT PUT NewSpeed=%f, New Turn Rate=%f, anglebetw=%f", robotSpeed, robotTurnRate, anglebetw);
  	qDebug ("=========================================================="); 
}




double ForceField::Delta_Angle(double a1, double a2) 
{//calculate angle start from a1 to a2
  double diff;
  diff = a2 - a1;
  if (diff > M_PI)
  {
    diff -= 2 * M_PI;
  } 
  else if (diff < -M_PI) 
  {
    diff += 2 * M_PI;
  }
  return(diff);
}


void ForceField::robotForceFieldShape(Robot * anotherrobot, QVector<QPointF> &Dmax_anotherrobot, QVector<QPointF> &Dmin_anotherrobot)
{
	//treated as circle
	//?????robots.x, robots.y, dynamicobstalce.phi, robotsRadius, robotsSpeed;
	double MaxSpeed_anotherrobot = 0.1, SysC_anotherrobot = 1, SysK_anotherrobot = SysK, FixedRatio_anotherrobot = 0.2;
	double angle_own = 0, number = 360, angleincr= 2 * M_PI / 360;
	//double number = ceil [2 * PI / angleincr];
 	double Er_anotherrobot = anotherrobot->robotSpeed / (MaxSpeed_anotherrobot  * SysC_anotherrobot);
 	//double Safedist_anotherrobot = 0.1;
 	//qDebug("HERE 3"); fflush(stdout);
 	//double Dmax_robots [number], Dmin_robots [number];
 	QPointF Temp1, Temp2;
 	double Dmax_anotherrobot_value, Dmin_anotherrobot_value; 
 	for (int i = 0; i < number; i++)
 	{
 		//qDebug("MaxSpeed_anotherrobot = %f, SysC_anotherrobot = %f, SysK_anotherrobot = %f, FixedRatio_anotherrobot = %f, Er_anotherrobot = %f", MaxSpeed_anotherrobot, SysC_anotherrobot, SysK_anotherrobot, FixedRatio_anotherrobot, Er_anotherrobot);
 		//qDebug("HERE 7, i = %d", i); fflush(stdout);
 		angle_own += angleincr;
 		//qDebug("HERE 4"); fflush(stdout);
 		//qDebug("HERE 5"); fflush(stdout);
 		//angle_anotherrobot = Delta_Angle(anotherrobot->robotLocation.phi, angle_own);
 		Dmax_anotherrobot_value = SysK_anotherrobot  * Er_anotherrobot  * anotherrobot->robotRadius / (1 - Er_anotherrobot * cos(angle_own));
 		Dmin_anotherrobot_value = FixedRatio_anotherrobot * Dmax_anotherrobot_value;
 		//qDebug("Dmax_anotherrobot_value = %f, Dmin_anotherrobot_value = %f", Dmax_anotherrobot_value, Dmin_anotherrobot_value);
 		//qDebug("HERE 6"); fflush(stdout);
 		
// 		Dmax_anotherrobot[i].setX (anotherrobot->robotLocation.p.x() + (Dmax_anotherrobot_value + anotherrobot->robotRadius) * cos(angle_own + anotherrobot->robotLocation.phi));
// 		Dmax_anotherrobot[i].setY (anotherrobot->robotLocation.p.y() + (Dmax_anotherrobot_value + anotherrobot->robotRadius) * sin(angle_own + anotherrobot->robotLocation.phi));
// 		Dmin_anotherrobot[i].setX (anotherrobot->robotLocation.p.x() + (Dmin_anotherrobot_value + anotherrobot->robotRadius) * cos(angle_own + anotherrobot->robotLocation.phi));
// 		Dmin_anotherrobot[i].setY (anotherrobot->robotLocation.p.y() + (Dmin_anotherrobot_value + anotherrobot->robotRadius) * sin(angle_own + anotherrobot->robotLocation.phi));
 		 
 		Temp1.setX (anotherrobot->robotLocation.p.x() + (Dmax_anotherrobot_value + anotherrobot->robotRadius) * cos(angle_own + anotherrobot->robotLocation.phi));
 		Temp1.setY (anotherrobot->robotLocation.p.y() + (Dmax_anotherrobot_value + anotherrobot->robotRadius) * sin(angle_own + anotherrobot->robotLocation.phi));
 		//qDebug ("%f     %f", Temp1.x(),Temp1.y());  /*****show Dmax*****/
 		Dmax_anotherrobot.push_back(Temp1);
 		Temp2.setX (anotherrobot->robotLocation.p.x() + (Dmin_anotherrobot_value + anotherrobot->robotRadius) * cos(angle_own + anotherrobot->robotLocation.phi));
 		Temp2.setY (anotherrobot->robotLocation.p.y() + (Dmin_anotherrobot_value + anotherrobot->robotRadius) * sin(angle_own + anotherrobot->robotLocation.phi));
 		//qDebug ("%f     %f", Temp2.x(),Temp2.y());
 		Dmin_anotherrobot.push_back(Temp2);
 		//qDebug ("Dmax_anotherrobotX=%f, Dmax_anotherrobotY=%f, Dmin_anotherrobotX=%f, Dmin_anotherrobotY=%f", Dmax_anotherrobot[i].x, Dmax_anotherrobot[i].y, Dmin_anotherrobot[i].x, Dmin_anotherrobot[i].y); 		
 	}
 	//qDebug("HERE 8"); fflush(stdout);
}

//double ForceField::ForceValue_anotherrobot(anotherrobot, anotherrobotDmaxPoint)
//{
//	double angle_anotherrobotDmax,closest_dist_anotherrobotDmax;
//	????closest_dist_anotherrobotDmax = Dist2Robot(obstacleDmaxPoint,angle_obstacleDmax);
//	double Er_anotherrobotDmax = anotherrobot->robotSpeed / (MaxSpeed * SysC);
// 	//double Safedist = 0.1;
// 	double Dmax_obstalceDmax = SysK * Er * robotRadius / (1 - Er_obstalceDmax * cos(angle_obstalceDmax));
// 	double Dmin_obstalceDmax = FixedRatio * Dmax_obstalceDmax;
// 	//qDebug ("Dmin=%f, Dmax=%f, clost_dist=%f", Dmin, Dmax, clostest_dist);
// 	double Ratio_obstalceDmax = closest_dist_obstalceDmax / Dmax_obstalceDmax;
// 	double ForceAmp;
// 	if (Ratio_obstalceDmax >= 1)
// 	{
//   		ForceAmp_obstalceDmax = 0.0;
// 	}
// 	else if (Ratio_obstalceDmax >= FixedRatio)
// 	{
//   		ForceAmp_obstalceDmax = SysP * (1 - Ratio) / (1 - FixedRatio);
// 	}
// 	else
// 	{
//   		ForceAmp_obstalceDmax = (1 - Ratio / 0.22) * SysP * 110;
// 	}
//    return(ForceAmp_obstalceDmax);
//	//printf ("ForceAmp=%f\n",ForceAmp);
//	//qDebug("Er:%f Angle:%f Dmax:%f Dmin:%f Ratio:%f ClosestDist:%f ForceAmp:%f",Er,angle,Dmax,Dmin,Ratio,closest_dist,ForceAmp); 	
//}

/******************************************************************/
void ForceField::VSFF(QVector<Interaction> obstacle_interaction_set, QVector<Interaction> robots_interaction_set, double realtime)
{
	qDebug ("VSFF!");
	qDebug ("OldSpeed=%f, OldDirec=%f, Turnate=%f, RR=%f", robotSpeed,robotLocation.phi,robotTurnRate, robotRadius);  
// 	double AbsSpeedX = robotSpeed * cos(robotLocation.phi);
// 	double AbsSpeedY = robotSpeed * sin(robotLocation.phi);
 	double TransformMatrix[2][2] = {{cos(robotLocation.phi), sin(robotLocation.phi)},
 									{-sin(robotLocation.phi), cos(robotLocation.phi)}}; //robotLocation.phi or -robotLocation.phi?
// 	//the attactive force
 	double FattAmp = SysQ;
 	double FattAngleA[2] = {goalLocation.p.x() - (robotLocation.p.x() + robotRadius * cos(robotLocation.phi)), goalLocation.p.y() - (robotLocation.p.y() + robotRadius * sin(robotLocation.phi))};
 	double FattAngleR[2];
 	MatrixMultipy(TransformMatrix, FattAngleA, FattAngleR);
 	double FattAngle = atan2(FattAngleR[1], FattAngleR[0]);
 	double FattX = FattAmp * cos(FattAngle);
 	double FattY = FattAmp * sin(FattAngle);
 	double FattArm[3] = {robotRadius, 0, 0};
 	double FattForce[3] = {FattX, FattY, 0};
 	double Matt[3];
 	CrossProduct (FattArm, FattForce, Matt);
 	
 	//qDebug ("X=%f, Y=%f, GoalX=%f, GoalY=%f", robotLocation.p.x(), robotLocation.p.y(), goalLocation.p.x(), goalLocation.p.y());
 	//qDebug ("FattAngleA_0=%f, FattAngleA_1=%f, FattAngleR_0=%f, FattAngleR_1=%f", FattAngleA[0], FattAngleA[1], FattAngleR[0], FattAngleR[1]);
 	//qDebug ("FattAmp = %f, FattAngle = %f, FattX = %f, FattY = %f, FattArm[0] = %f, FattArm[1] = %f, Matt[2] = %f\n",FattAmp, FattAngle, FattX, FattY, FattArm[0], FattArm[1], Matt[2]);
 	double MrepTotal = 0, FrepXTotal = 0, FrepYTotal = 0;
 	for(int i = 0; i<obstacle_interaction_set.size();i++)
 	{
   		double FrepAmp   = obstacle_interaction_set[i].force;
   		double FrepAngle = obstacle_interaction_set[i].direction - robotLocation.phi;
   		double FrepX = FrepAmp * cos(FrepAngle);
   		double FrepY = FrepAmp * sin(FrepAngle);
   		double FrepArmA[2] = {obstacle_interaction_set[i].location.x() - robotLocation.p.x(), obstacle_interaction_set[i].location.y() - robotLocation.p.y()};
  	 	double FrepArmR[2];
   		MatrixMultipy(TransformMatrix, FrepArmA, FrepArmR);
   		double FrepArm[3] = {FrepArmR[0], FrepArmR[1], 0};
  		double FrepForce[3] = {FrepX, FrepY, 0};
   		double Mrep[3];
   		CrossProduct (FrepArm, FrepForce, Mrep);
  		//qDebug ("ObstacleX=%f, ObstacleY=%f, FrepAmp = %f, FrepAngle = %f, FrepX = %f, FrepY = %f, FrepArm[0] = %f, FrepArm[1] = %f, Mrep[2] = %f", obstacle_interaction_set[i].location.x(), obstacle_interaction_set[i].location.y(), FrepAmp, FrepAngle, FrepX, FrepY, FrepArm[0], FrepArm[1], Mrep[2]);
  		FrepXTotal = FrepXTotal + FrepX;
		FrepYTotal = FrepYTotal + FrepY;
   		MrepTotal = MrepTotal + Mrep[2];
 	}
 	//qDebug ("FrepXTotal = %f, FrepYTotal = %f, MrepTotal = %f\n", FrepXTotal, FrepYTotal, MrepTotal);
 	double FtotalX = FattX + FrepXTotal;
 	double FtotalY = FattY + FrepYTotal;
 	double Mtotal = Matt[2] + MrepTotal;
 	qDebug ("FtotalX = %f, FtotalY = %f, Mtotal = %f\n", FtotalX, FtotalY, Mtotal);
 	double AcceRX = FtotalX; // robotMass;
 	double AcceRY = FtotalY;// robotMass;
 	double SpeedX = robotSpeed + AcceRX * TimeStep;
	double SpeedY = AcceRY * TimeStep;
	qDebug ("AcceRX=%f, AcceRY=%f, SpeedX=%f, SpeedY=%f",AcceRX, AcceRY, SpeedX, SpeedY);
	robotSpeed = sqrt(SpeedX * SpeedX + SpeedY * SpeedY);
	double AcceRA = atan2(SpeedY, SpeedX);
	qDebug ("AcceRA=%f", AcceRA);
	double Omegadot = Mtotal / 1;
	qDebug ("Omegadot=%f", Omegadot);
	double robotTurnRate1 = robotTurnRate + Omegadot * TimeStep;
	qDebug ("robotTurnRate1=%f", robotTurnRate1);
	robotTurnRate1 = robotTurnRate1 + AcceRA * TimeStep;
	qDebug ("robotTurnRate1=%f", robotTurnRate1);
	double Limit = 0.1;
	if ((robotTurnRate1 - robotTurnRate) > Limit)
	{
		robotTurnRate = robotTurnRate + Limit;
	}
	else if ((robotTurnRate1 - robotTurnRate) < -Limit)
	{
		robotTurnRate = robotTurnRate - Limit;
	}
	else
	{
	robotTurnRate = robotTurnRate1;
	}
	//qDebug ("robotTurnRate=%f", robotTurnRate);
	
 	if (SpeedX > MaxSpeed)
 	{
  		robotSpeed = MaxSpeed;
 	}
 	else if (SpeedX < 0)
 	{
  		robotSpeed = 0.01;
 	}
 	if (robotTurnRate > OmegaMax)
 	{
  		robotTurnRate = OmegaMax;
 	}
 	else if (robotTurnRate < -OmegaMax)
 	{
   		robotTurnRate = -OmegaMax;
	}
	qDebug ("Omegadot=%f, robotTurnRate=%f,robotSpeed=%f, MaxSpeed=%f", Omegadot, robotTurnRate, robotSpeed, MaxSpeed);	
}


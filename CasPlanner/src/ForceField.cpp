#include "ForceField.h"

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
velVector ForceField::GenerateField(Pose pose,QVector<QPointF> laser_set,Pose Goal,double speed,double turnrate)
{
	velVector action;
 	this->robotLocation = pose;
 	this->goalLocation  = Goal;
 	this->robotSpeed = speed;
 	this->robotTurnRate = turnrate;	 	
 	double coefficient[curvefittingorder];
	QVector< QVector<QPointF> > obstacles_set = DivObst(laser_set,Pose(0,0,0));
	QVector<Interaction> obstacle_interaction_set;
	qDebug ("obstacles_set.size()=%d",obstacles_set.size()); 
 	for (int i = 0; i <obstacles_set.size() ; i++)
 	{
 		Interaction inter_point, max_inter;
 		max_inter.force = -100;
 		double sum_x = 0,average_x;
 		int j;
		for (j =0; j < obstacles_set[i].size();j++)
 		{
 			inter_point.location.setX(obstacles_set[i][j].x());
 			inter_point.location.setY(obstacles_set[i][j].y());	 			
 			inter_point.force = ForceValue(inter_point.location,Pose(0,0,0));
 			sum_x += inter_point.location.x();
 			if(inter_point.force > max_inter.force )
 			{
 				max_inter = inter_point;
 				inter_point.direction = 0;
 			}
 		}
 		average_x = sum_x/(j+1);		
	   	if (!IsZero(max_inter.force))
	   	{
	   		qDebug("MAX INTER FORCE:%f",max_inter.force); 	   		
     		LSCurveFitting(obstacles_set[i],coefficient,curvefittingorder);
     		double DiffInterPoint = coefficient[1], Xoffset = max_inter.location.x() - average_x;
     		for (int diffo = 2; diffo < curvefittingorder; diffo++)
     		{
       			DiffInterPoint = diffo * coefficient[diffo] * pow(Xoffset, diffo - 1) + DiffInterPoint;
     		}
     		double NormInterPoint = FindNorm(max_inter.location,DiffInterPoint);
     		max_inter.direction = NormInterPoint;
			obstacle_interaction_set.push_back(max_inter);
	   	}
	 }
	if(FF_algorithm == VariableSpeedFF)
	{
		//qDebug("Obstacle Interaction Set Before VSFF=%d",obstacle_interaction_set.size());
 		VSFF(obstacle_interaction_set);
	}
	else
	{
		SimFF(obstacle_interaction_set);
	}
 	action.speed= robotSpeed;
 	action.turnRate = robotTurnRate;
 	return action;
};
double ForceField::FindNorm(QPointF interaction_point, double Tang)
{
	//DotMultiply(QPointF p1,QPointF p2,QPointF p0)
	double NormDir, Norm;
	if (IsZero(Tang))
	{
		Norm = INF;
	}
	else
	{
		Norm = 1 / Tang;
	}
	QPointF p1, p2, pR;
	pR.setX(robotLocation.p.x()); pR.setY(robotLocation.p.y());
	p1.setX(interaction_point.x() + 1); p1.setY(interaction_point.y() + Norm);
	p2.setX(interaction_point.x() - 1); p2.setY(interaction_point.y() - Norm);
	qDebug ("Tang=%f, p1X=%f, p1Y=%f, p2X=%f, p2Y=%f, pRX=%f, pRY=%f", Tang, p1.x(), p1.y(), p2.x(), p2.y(), pR.x(), pR.y());
	double Value1 = DotMultiply(p1, pR, interaction_point);
	double Value2 = DotMultiply(p2, pR, interaction_point);
	if (Value1 > 0)
	{
		NormDir = ATAN2(p1, interaction_point);
	}
	else if (Value1 < 0)
	{
		NormDir = ATAN2(p2, interaction_point);
	}
	else
	{
		NormDir = M_PI / 2;
	}
	qDebug ("Value1=%f, Value2=%f, NormDir=%f", Value1, Value2, NormDir);
	return(NormDir);
}

double ForceField::ForceValue(QPointF ray_end,Pose laser_pose)
{
 	double MinSpeed = 0.05;
 	if ((robotSpeed < MinSpeed) && (robotSpeed >= 0))
 	{
 		robotSpeed = MinSpeed;
 	}
 	if ((robotSpeed > - MinSpeed) && (robotSpeed < 0))
 	{
 		robotSpeed = - MinSpeed;
 	}
	double angle,closest_dist;
	closest_dist = Dist2Robot(ray_end,angle);
 	double Er = fabs(robotSpeed) / (MaxSpeed * SysC);
 	double Dmax = SysK * Er * robotRadius / (1 - Er * cos(angle));
 	double Dmin = FixedRatio * Dmax;
 	double Ratio = closest_dist / Dmax;
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
   		ForceAmp = SysP * 10;
 	}
 	//printf ("ForceAmp=%f\n",ForceAmp);
	qDebug("Er:%f Angle:%f Dmax:%f Dmin:%f Ratio:%f ClosestDist:%f ForceAmp:%f",Er,angle,Dmax,Dmin,Ratio,closest_dist,ForceAmp); 	
 	return(ForceAmp);
}

QVector < QVector<QPointF> > ForceField::DivObst(QVector<QPointF> laser_set,Pose laser_pose)
{
 	double DistBetw;
 	QPointF p1,p2;
	QVector < QVector<QPointF> > obstacles_set;
	QVector<QPointF> obstacle;
	p1.setX(laser_set[0].x())	; p1.setY(laser_set[0].y());		
	obstacle.push_back(p1);		
	for (int i = 0; i < laser_set.size() - 1; i++)
 	{
		p1.setX(laser_set[i].x())	; p1.setY(laser_set[i].y());
		p1 = Trans2Global(p1,laser_pose);p1 = Trans2Global(p1,robotLocation);
		p2.setX(laser_set[i+1].x())	; p2.setY(laser_set[i+1].y());	
		p2 = Trans2Global(p2,laser_pose);p2 = Trans2Global(p2,robotLocation);				
   		DistBetw = Dist(p1,p2);
   		if (DistBetw < Gapdist)
   		{
     		obstacle.push_back(p1);
   		}
   		else
   		{
   			obstacles_set.push_back(obstacle);
   			obstacle.clear();
   			obstacle.push_back(p2);
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
 	double z, p, c, g, q, d1, d2, s[20], t[20], b[20];
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
void ForceField::VSFF(QVector<Interaction> obstacle_interaction_set)
{
	qDebug ("VSFF!");
 	double AbsSpeedX = robotSpeed * cos(robotLocation.phi);
 	double AbsSpeedY = robotSpeed * sin(robotLocation.phi);
 	double TransformMatrix[2][2] = {{cos(robotLocation.phi), sin(robotLocation.phi)},
 									{-sin(robotLocation.phi), cos(robotLocation.phi)}}; //robotLocation.phi or -robotLocation.phi?
 	//the attactive force
 	double FattAmp = SysQ;
 	double FattAngleA[2] = {goalLocation.p.x() - (robotLocation.p.x() + 0.1 * robotRadius * cos(robotLocation.phi)), goalLocation.p.y() - (robotLocation.p.y() + 0.1 * robotRadius * sin(robotLocation.phi))};
 	double FattAngleR[2];
 	MatrixMultipy(TransformMatrix, FattAngleA, FattAngleR);
 	double FattAngle = atan2(FattAngleR[1], FattAngleR[0]);
 	double FattX = FattAmp * cos(FattAngle);
 	double FattY = FattAmp * sin(FattAngle);
 	double FattArm[3] = {robotRadius, 0, 0};
 	double FattForce[3] = {FattX, FattY, 0};
 	double Matt[3];
 	CrossProduct (FattArm, FattForce, Matt);
 	//printf ("\n");
 	qDebug ("OldSpeed=%f, OldDirec=%f, Turnate=%f, RR=%f", robotSpeed,robotLocation.phi,robotTurnRate, robotRadius);  
 	qDebug ("X=%f, Y=%f, GoalX=%f, GoalY=%f", robotLocation.p.x(), robotLocation.p.y(), goalLocation.p.x(), goalLocation.p.y());
 	qDebug ("FattAngleA_0=%f, FattAngleA_1=%f, FattAngleR_0=%f, FattAngleR_1=%f", FattAngleA[0], FattAngleA[1], FattAngleR[0], FattAngleR[1]);
 	qDebug ("FattAmp = %f, FattAngle = %f, FattX = %f, FattY = %f, FattArm[0] = %f, FattArm[1] = %f, Matt[2] = %f\n",FattAmp, FattAngle, FattX, FattY, FattArm[0], FattArm[1], Matt[2]);
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
   		//printf ("\n");
   		qDebug ("ObstacleX=%f, ObstacleY=%f, FrepAmp = %f, FrepAngle = %f, FrepX = %f, FrepY = %f, FrepArm[0] = %f, FrepArm[1] = %f, Mrep[2] = %f", obstacle_interaction_set[i].location.x(), obstacle_interaction_set[i].location.y(), FrepAmp, FrepAngle, FrepX, FrepY, FrepArm[0], FrepArm[1], Mrep[2]);
   		FrepXTotal = FrepXTotal + FrepX;
		FrepYTotal = FrepYTotal + FrepY;
   		MrepTotal = MrepTotal + Mrep[2];
   		//sleep(10);
 	}

 	double FtotalX = FattX + FrepXTotal;
 	double FtotalY = FattY + FrepYTotal;
 	double Mtotal = Matt[2] + MrepTotal;
 	qDebug ("FtotalX = %f, FtotalY = %f, Mtotal = %f\n", FtotalX, FtotalY, Mtotal);
 	double AcceRX = FtotalX / robotMass;
 	double AcceRY = FtotalY / robotMass;

 	double AcceR = sqrt(AcceRX * AcceRX + AcceRY * AcceRY);
 	double AcceRA = atan2 (AcceRY, AcceRX);
 	qDebug ("AcceR=%f, AcceRX=%f, AcceRY=%f, AcceRA=%f, MaxAcceT=%f", AcceR, AcceRX, AcceRY, AcceRA, MaxAcceT);
 	if (AcceR > MaxAcceT)
 	{
   		AcceR = MaxAcceT;
   		AcceRX = MaxAcceT * cos(AcceRA);
   		AcceRY = MaxAcceT * sin(AcceRA);
 	}
 	else if (AcceR < -MaxAcceT)
 	{
   		AcceR = -MaxAcceT;
   		AcceRX = -MaxAcceT * cos(AcceRA);
   		AcceRY = -MaxAcceT * sin(AcceRA);
 	}
 	qDebug ("AcceRX=%f, AcceRY=%f", AcceRX, AcceRY);
 	//double J = 0.5 * robotMass * robotRadius * robotRadius;
	double Omegadot = Mtotal / robotMI;
	qDebug ("Omegadot=%f", Omegadot);
 	if (Omegadot > OmegadotMax)
 	{
   		Omegadot = OmegadotMax;
 	}
 	else if (Omegadot < -OmegadotMax)
 	{
  		Omegadot = -OmegadotMax;
 	}
 	robotTurnRate = robotTurnRate + Omegadot * TimeStep;
 	qDebug ("robotTurnRate=%f", robotTurnRate);
 	if (robotTurnRate > OmegaMax)
 	{
  		robotTurnRate = OmegaMax;
 	}
 	else if (robotTurnRate < -OmegaMax)
 	{
   		robotTurnRate = -OmegaMax;
 	}
 	robotLocation.phi = robotLocation.phi + robotTurnRate * TimeStep; //

 	double AcceAX = cos(robotLocation.phi) * AcceRX - sin(robotLocation.phi) * AcceRY; //robotLocation.phi or -robotLocation.phi?
 	double AcceAY = sin(robotLocation.phi) * AcceRX + cos(robotLocation.phi) * AcceRY;

 	double AbsSpeedX1 = AbsSpeedX + AcceAX * TimeStep;
 	double AbsSpeedY1 = AbsSpeedY + AcceAY * TimeStep;

 	robotSpeed = sqrt(AbsSpeedX1 * AbsSpeedX1 + AbsSpeedY1 * AbsSpeedY1);
 	qDebug ("robotSpeed=%f", robotSpeed);
 	robotLocation.phi = atan2(AbsSpeedY1, AbsSpeedX1);

 	if (robotSpeed > MaxSpeed)
 	{
  		robotSpeed = MaxSpeed;
   		AbsSpeedX1 = robotSpeed * cos(robotLocation.phi);
   		AbsSpeedY1 = robotSpeed * sin(robotLocation.phi);
 	}
 	else if (robotSpeed < -MaxSpeed)
 	{
   		robotSpeed = -MaxSpeed;
   		AbsSpeedX1 = -robotSpeed * cos(robotLocation.phi);
   		AbsSpeedY1 = -robotSpeed * sin(robotLocation.phi);
 	}
 	
 	double MinSpeed = 0.05;
 	if ((robotSpeed < MinSpeed) && (robotSpeed >= 0))
 	{
 		robotSpeed = MinSpeed;
 		AbsSpeedX1 = robotSpeed * cos(robotLocation.phi);
   		AbsSpeedY1 = robotSpeed * sin(robotLocation.phi);
 	}
 	if ((robotSpeed > - MinSpeed) && (robotSpeed < 0))
 	{
 		robotSpeed = - MinSpeed;
 		AbsSpeedX1 = -robotSpeed * cos(robotLocation.phi);
   		AbsSpeedY1 = -robotSpeed * sin(robotLocation.phi);
 	}
	qDebug ("Omegadot=%f, OmegadotMax=%f, robotTurnRate=%f, OmegaMax=%f, robotSpeed=%f, MaxSpeed=%f", Omegadot, OmegadotMax, robotTurnRate, OmegaMax, robotSpeed, MaxSpeed);	
}
void ForceField::SimFF(QVector<Interaction> obstacle_interaction_set)
{
	
	qDebug ("Simple FF!");
	qDebug ("OldSpeed=%f, OldDirec=%f", robotSpeed,robotLocation.phi);  
	double FattAmp = SysQ;
  	double FattAngleA[2] = {goalLocation.p.x() - robotLocation.p.x(), goalLocation.p.y() - robotLocation.p.y()};
  	double FattAngle = atan2(FattAngleA[1], FattAngleA[0]);
  	double FattX = FattAmp * cos(FattAngle);
  	double FattY = FattAmp * sin(FattAngle);
  	qDebug ("X=%f, Y=%f, GoalX=%f, GoalY=%f", robotLocation.p.x(), robotLocation.p.y(), goalLocation.p.x(), goalLocation.p.y());
  	qDebug ("FattAmp = %f, FattAngle = %f, FattX = %f, FattY = %f",FattAmp, FattAngle, FattX, FattY);
  	double FrepXTotal = 0, FrepYTotal = 0;
  	for(int i = 0; i<obstacle_interaction_set.size();i++)
  	{
    	double FrepAmp   = obstacle_interaction_set[i].force;
    	double FrepAngle = obstacle_interaction_set[i].direction;
    	double FrepX = FrepAmp * cos(FrepAngle);
    	double FrepY = FrepAmp * sin(FrepAngle);
    	qDebug ("ObstacleX=%f, ObstacleY=%f, FrepAmp = %f, FrepAngle = %f, FrepX = %f, FrepY = %f", obstacle_interaction_set[i].location.x(), obstacle_interaction_set[i].location.y(), FrepAmp, FrepAngle, FrepX, FrepY);
    	FrepXTotal = FrepXTotal + FrepX;
    	FrepYTotal = FrepYTotal + FrepY;
  	}
  	double FtotalX = FattX + FrepXTotal;
  	double FtotalY = FattY + FrepYTotal;
  	double ForceAngle = atan2(FtotalY, FtotalX);
  	qDebug ("FtotalX=%f, FtotalY=%f, ForceAngle=%f", FtotalX, FtotalY, ForceAngle * 180 / M_PI);
  	robotSpeed = 0.1;
  	robotTurnRate = (ForceAngle - robotLocation.phi) / TimeStep;
  	qDebug ("robotTurnRate=%f", robotTurnRate);  
  	double TurnLimit = 1;
  	if (robotTurnRate > TurnLimit)
  	{
  		robotTurnRate = TurnLimit;
  	}
  	else if (robotTurnRate < -TurnLimit)
  	{
  		robotTurnRate = - TurnLimit;
  	}
  	robotLocation.phi = robotLocation.phi + robotTurnRate * TimeStep;
  	//robotLocation.phi = ForceAngle;
  	qDebug ("NewSpeed=%f, NewDirec=%f, robotTurnRate=%f", robotSpeed,robotLocation.phi, robotTurnRate);  
  	//return(ForceAngle);
}

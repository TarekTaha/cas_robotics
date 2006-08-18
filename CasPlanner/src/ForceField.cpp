#include "ForceField.h"
//Empty constructor
namespace CasPlanner
{
	ForceField::ForceField()
	{
	}
	// Constructor with the Robot Size
	ForceField::ForceField(Robot * robot)
	{
	}
	// Destructor -> Free Memory
	ForceField::~ForceField()
	{
	}
	/* Given a Position and an environment, generate the action to 
	 * go to the goal Point without colliding with the surrondings
	 */
	double ForceField::NearestObstacle(QVector<QPointF> laser_scan,Pose lase_pose)
	{
		QPointF ray_end,temp[4],intersection;
		Line L1,L2;
		double dist,shortest_dist=10000;
		for(int i=0;i<4;i++)
		{
			temp[i] = local_planner->pathPlanner->local_edge_points[i];
		}
		for(int i=0;i<laser_scan.size();i++)
		{
			ray_end = Trans2Global(laser_scan[i],laser_pose);
	//		ray_end = laser_scan[i];
			for(int j=0;j<4;j++)
			{
				L1.SetStart(temp[j%4]);      L1.SetEnd(temp[(j+1)%4]);
				L2.SetStart(laser_pose.p);   L2.SetEnd(ray_end);
				if(LineInterLine(L1,L2,intersection))
				{
					dist = Dist(intersection,ray_end);
					if(dist < shortest_dist)
					{
						shortest_dist = dist;
					}
				}
			}
		}
		return shortest_dist;	
	};
	VelVector ForceField::GenerateField(QPointF position,QVector<QPointF> laser_set,QPointF Goal,QPointF location,double speed,double turnrate)
	{
	 	this->robotLocation = location;
	 	this->goalLocation  = Goal;
	 	double x, y, TempX[2000], TempY[2000];
	 	double coefficient[curvefittingorder], dt[3];
	 	char DSL_a [50][80];
	 	int DSL_c = 3000;
	 	double DSL_d = Gapdist;
	 	int DSL_FN = 0;
	 	int DSL_e = DataSortingLaser(QVector<QPointF> laser_set);
	 	//printf ("\nDSL_e=  %d\n",DSL_e);
	 	double InterActPoint[DSL_e][4];
	 	int CP = 0;
	 	for (int FN = 0; FN <= (DSL_e - 1); FN++)
	 	{
	   		//printf("\n");
	   		//printf ("%s\n",DSL_a[FN]);
	   		//read data from file;
		   	if ((PointerReadFromSortedFile = fopen(DSL_a[FN], "r")) == NULL)
		   	{
		   		printf ("Cannot open file!\n");
		   	}
		   	int Cdata = 0;
	   	  	while (! feof ( PointerReadFromSortedFile ))
	   	  	{
	     		//fscanf (PointerReadFromSortedFile,"%lf     %lf     %lf\n", &x, &y, &z);
	     		fscanf (PointerReadFromSortedFile,"%lf     %lf\n", &x, &y);
	     		TempX[Cdata] = x;
	     		TempY[Cdata] = y;
	    		//TempZ[Cdata] = z;
	     		Cdata++;
	   		}
	   		fclose (PointerReadFromSortedFile);
	   		double DataX[Cdata], DataY[Cdata], DataZ[Cdata];
	   		double InterActX = 0, InterActY = 0, InterActZ = EP;
	   		for (int Ctmp = 0; Ctmp <= Cdata - 1; Ctmp++)
	   		{
	     		DataX[Ctmp] = TempX[Ctmp];
	     		DataY[Ctmp] = TempY[Ctmp];
	     		DataZ[Ctmp] = ForceValue(DataX[Ctmp], DataY[Ctmp], RobInfo, SysInfo, Vertex);
	     		//printf ("X=%f,Y=%f,Z=%f,Ctmp=%d\n", DataX[Ctmp], DataY[Ctmp], DataZ[Ctmp],Ctmp);
	     		if (DataZ[Ctmp] > InterActZ)
	     		{
	       			InterActX = DataX[Ctmp];InterActY = DataY[Ctmp];InterActZ = DataZ[Ctmp]; //Interaction Point
	     		}
	   		}
	   		//printf("\n");
	   		//printf ("the Possible Interaction Point is:\n");
	   		//printf ("X=%f,Y=%f,Z=%f\n", InterActX, InterActY, InterActZ);
	   		//printf("\n");
		   	if (! ((InterActZ > -EP) && (InterActZ < EP)))
		   	{
		     		LSCurveFitting (DataX, DataY, Cdata - 1, coefficient, curvefittingorder, dt);
		     		double Xsum = 0;
		     		for (int Cave = 0; Cave <= Cdata - 1; Cave++)
		     		{
		       			Xsum= Xsum + DataX[Cave];
		     		}
		     		double Xave = Xsum / Cdata;
		     		//printf ("ave=%f\n",Xave);
		     		double DiffInterPoint = coefficient[1], Xoffset = InterActX - Xave;
		     		for (int diffo = 2; diffo < curvefittingorder; diffo++)
		     		{
		       			DiffInterPoint = diffo * coefficient[diffo] * pow(Xoffset, diffo - 1) + DiffInterPoint;
		     		}
		     		double NormInterPoint = FindNorm(InterActX, InterActY, DiffInterPoint, RX, RY);
		     		//printf ("X = %f, Y = %f, Diff at the Interaction Point is: %f\nNorm at the Interaction Point is: %f\nCP = %d\n", InterActX, InterActY, DiffInterPoint, NormInterPoint*180/PI, CP);
		     		//InterActPoint[CP][0] = {InterActX, InterActY, InterActZ, NormInterPoint};
		     		InterActPoint[CP][0] = InterActX;
		     		InterActPoint[CP][1] = InterActY;
		     		InterActPoint[CP][2] = InterActZ;
		     		InterActPoint[CP][3] = NormInterPoint;
		     		CP++;
		     		//printf ("CP = %d\n", CP);
		   	}
		 }
	 	//RobInfo;// = {RX, RY, RR, robotSpeed, robotLocation.phi, robotMass, robotMI, GoalX, goalLocation.p.y(), Omega};
	 	//SysInfo;// = {TimeStep, SysK, SysC, SysFR, SysP, SysQ, MaxSpeed, MaxAcceT, OmegadotMax, OmegaMax};
	 	VSFF(RobInfo, SysInfo, InterActPoint, CP);
	 	//printf ("\n");
	 	//printf ("RX = %f, RY = %f, robotSpeed = %f, robotLocation.phi = %f, Omega = %f\n", RobInfo[0], RobInfo[1], RobInfo[3], RobInfo[4], RobInfo[9]);
	}
	double ForceValue(double PointX, double PointY, double RobotInfo[], double SysPara[], POINT RobotVertex[])
	{
		double RobotX = RobotInfo[0];
	 	double RobotY = RobotInfo[1];
	 	double RobotRadius = RobotInfo[2];
		double RobotSpeed = RobotInfo[3];
	 	double RobotDirection = RobotInfo[4];
	
	 	double SysK = SysPara[1];
	 	double SysC = SysPara[2];
	 	double FixedRatio = SysPara[3];
	 	double SysP = SysPara[4];
	 	double MaxSpeed = SysPara[6];
	
	 	POINT P;
		P.x = PointX;
	 	P.y = PointY;
	 	POINT NP;
	 	double Dist = PtoRobotDist(RobotVertex, P, NP);
	
	 	//start line
	 	POINT ss = {RobotX,RobotY};
	 	POINT se = {RobotX + RobotRadius * cos(RobotDirection),RobotY + RobotRadius * sin(RobotDirection)};
	 /	/end line
	
	 	POINT es = NP;
	 	POINT ee = P;
	
	 	LINESEG start, end;
	 	start.s = ss;
	 	start.e = se;
		end.s = es;
	 	end.e = ee;
	
	 	double CosTheta=Cosine(end,start);
	 	//printf ("f=%f,    ",CosTheta);
	
	 	double Er = RobotSpeed / (MaxSpeed * SysC);
	 	double Dmax = SysK * Er * RobotRadius / (1 - Er * CosTheta);
	 	double Dmin = FixedRatio * Dmax;
	 	double Ratio = Dist / Dmax;
	
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
	 	return(ForceAmp);
	 	//double ForceDirection
		}

	QVector<QPointF> * DataSortingLaser(QVector<QPointF> laser_set)
	{
	 	double DistBetw;
	 	QPointF p1,p2;
		int MAXOBSTACLES = 20;
		QVector<QPointF> obstacles_set[MAXOBSTACLES];
		int obst_count=0;
		p1.setX(laser_set[0].x())	; p1.setY(laser_set[0].y());		
   		obstacles_set[obst_count].push_back(p1);		
		for (int i = 0; i <= laser_set.size() - 1; i++)
	 	{
			p1.setX(laser_set[i].x())	; p1.setY(laser_set[i].y());
			p2.setX(laser_set[i].x())	; p2.setY(laser_set[i].y());			
	   		DistBetw = Dist(p1,p2);
	   		if (DistBetw < GapDist)
	   		{
	     		obstacles_set[obst_count].push_back(p1);
	   		}
	   		else
	   		{
	   			obst_count++;
	   			obstacles_set[obst_count].push_back(p2);	   		
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
	void LSCurveFitting (QVector<QPointF> obstacle, double a[], int m, double dt[])
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
	 	dt[0] = 0.0;
	 	dt[1] = 0.0;
	 	dt[2] = 0.0;
	 	for (i = 0; i <= (n - 1); i++)
	 	{
	   		q = a[m - 1];
	   		for (k = m - 2; k >= 0; k--)
	     		q = a[k] + q * (obstacle[i].x()- z);
	     	double PtoRobotDist(POINT[], POINT);
		   	p = q - obstacle[i].y();
	   		if (fabs(p) > dt[2])
	    		dt[2] = fabs(p);
	   		dt[0] = dt[0] + p * p;
	   		dt[1] = dt[1] + fabs (p);
	 	}
	 	return;
	}

	/*****************************************************************/
	void VSFF(double InterActPoint[][4], int IP)
	{
	 	double AbsSpeedX = robotSpeed * cos(robotLocation.phi);
	 	double AbsSpeedY = robotSpeed * sin(robotLocation.phi);
	 	double TransformMatrix[2][2] = {{cos(robotLocation.phi), sin(robotLocation.phi)},
	 									{-sin(robotLocation.phi), cos(robotLocation.phi)}}; //robotLocation.phi or -robotLocation.phi?
	 	//the attactive force
	 	double FattAmp = SysQ;
	 	double FattAngleA[2] = {goalLocation.p.x() - (center.x() + robotRadius * cos(robotLocation.phi)), goalLocation.p.y() - (center.y() + robotRadius * sin(robotLocation.phi))};
	 	double FattAngleR[2];
	 	MatrixMultipy(TransformMatrix, FattAngleA, FattAngleR);
	 	double FattAngle = FindAngle(FattAngleR[0], FattAngleR[1]);////
	 	double FattX = FattAmp * cos(FattAngle);
	 	double FattY = FattAmp * sin(FattAngle);
	 	double FattArm[3] = {robotRadius, 0, 0};
	 	double FattForce[3] = {FattX, FattY, 0};
	 	double Matt[3];
	 	CrossProduct (FattArm, FattForce, Matt);
	 	//printf ("\n");
	 	//printf ("FattAmp = %f, FattAngle = %f, FattX = %f, FattY = %f, FattArm[0] = %f, FattArm[1] = %f, Matt[2] = %f\n",FattAmp, FattAngle, FattX, FattY, FattArm[0], FattArm[1], Matt[2]);
	 	double MrepTotal = 0, FrepXTotal = 0, FrepYTotal = 0;
	 	for (int Cpoints = 0; Cpoints <= IP - 1; Cpoints++)
	 	{
	   		double InterActX = InterActPoint[Cpoints][0];
	   		double InterActY = InterActPoint[Cpoints][1];
	   		double FrepAmp = InterActPoint[Cpoints][2];
	   		double FrepAngle = InterActPoint[Cpoints][3] - robotLocation.phi;
	   		double FrepX = FrepAmp * cos(FrepAngle);
	   		double FrepY = FrepAmp * sin(FrepAngle);
	
	   		double FrepArmA[2] = {InterActX - center.x(), InterActY - center.y()};
	  	 	double FrepArmR[2];
	   		MatrixMultipy(TransformMatrix, FrepArmA, FrepArmR);
	   		double FrepArm[3] = {FrepArmR[0], FrepArmR[1], 0};
	  		 double FrepForce[3] = {FrepX, FrepY, 0};
	   		double Mrep[3];
	   		CrossProduct (FrepArm, FrepForce, Mrep);
	   		//printf ("\n");
	   		//printf ("FrepAmp = %f, FrepAngle = %f, FrepX = %f, FrepY = %f, FrepArm[0] = %f, FrepArm[1] = %f, Mrep[2] = %f\n",FrepAmp, FrepAngle, FrepX, FrepY, FrepArm[0], FrepArm[1], Mrep[2]);
	   		FrepXTotal = FrepXTotal + FrepX;
			FrepYTotal = FrepYTotal + FrepY;
	   		MrepTotal = MrepTotal + Mrep[2];
	 	}
	
	 	double FtotalX = FattX + FrepXTotal;
	 	double FtotalY = FattY + FrepYTotal;
	 	double Mtotal = Matt[2] + MrepTotal;
	 	//printf ("\n");
	 	//printf ("FtotalX = %f, FtotalY = %f, Mtotal = %f\n", FtotalX, FtotalY, Mtotal);
	
	 	double AcceRX = FtotalX / robotMass;
	 	double AcceRY = FtotalY / robotMass;
	
	 	double AcceR = sqrt(AcceRX * AcceRX + AcceRY * AcceRY);
	 	double AcceRA = FindAngle (AcceRX, AcceRY);
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
	 	//double J = 0.5 * robotMass * robotRadius * robotRadius;
		 double Omegadot = Mtotal / robotMI;
	 	if (Omegadot > OmegadotMax)
	 	{
	   		Omegadot = OmegadotMax;
	 	}
	 	else if (Omegadot < -OmegadotMax)
	 	{
	  		Omegadot = -OmegadotMax;
	 	}
	
	 	robotTurnRate = robotTurnRate + Omegadot * Timestep;
	 	if (robotTurnRate > OmegaMax)
	 	{
	  		robotTurnRate = OmegaMax;
	 	}
	 	else if (robotTurnRate < -OmegaMax)
	 	{
	   		robotTurnRate = -OmegaMax;
	 	}
	
	 	robotLocation.phi = robotLocation.phi + robotTurnRate * Timestep; //
	
	 	double AcceAX = cos(robotLocation.phi) * AcceRX - sin(robotLocation.phi) * AcceRY; //robotLocation.phi or -robotLocation.phi?
	 	double AcceAY = sin(robotLocation.phi) * AcceRX + cos(robotLocation.phi) * AcceRY;
	
	 	double AbsSpeedX1 = AbsSpeedX + AcceAX * Timestep;
	 	double AbsSpeedY1 = AbsSpeedY + AcceAY * Timestep;
	
	 	robotSpeed = sqrt(AbsSpeedX1 * AbsSpeedX1 + AbsSpeedY1 * AbsSpeedY1);
	 	robotLocation.phi = FindAngle (AbsSpeedX1, AbsSpeedY1);
	
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
	
	 	//RX = RX + (AbsSpeedX + AbsSpeedX1) * Timestep / 2;
	 	//RY = RY + (AbsSpeedY + AbsSpeedY1) * Timestep / 2;
	
	 	//RobInfo;
	
	 	//RobInfo[0] = RX;
		//RobInfo[1] = RY;
	 	//RobInfo[2] = robotRadius;
	 	RobInfo[3] = robotSpeed;
	 	RobInfo[4] = robotLocation.phi;
	 	//RobInfo[5] = robotMass;
	 	//RobInfo[6] = robotMI;
	 	//RobInfo[7] = goalLocation.p.x();
	 	//RobInfo[8] = goalLocation.p.y();
	 	RobInfo[9] = robotTurnRate;
		}

};

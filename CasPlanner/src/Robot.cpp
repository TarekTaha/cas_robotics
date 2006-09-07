#include "Robot.h"

Robot::Robot(ConfigFile *cf,int secId)
{
	readConfigs(cf,secId);
	findR();
};
int Robot::readConfigs(ConfigFile *cf,int secId)
{
	int cnt;
	// Read Parameters of interest
   	robotName  =           cf->ReadString(secId, "robotName", "No-Name");
   	robotModel =           cf->ReadString(secId, "robotModel", "Diff");		   	
	robotIp =              cf->ReadString(secId, "robotIp", "127.0.0.1");
	robotPort =            cf->ReadInt   (secId, "robotPort", 6665);
	robotLength =          cf->ReadFloat (secId, "robotLength", 6665);			
	robotWidth =           cf->ReadFloat (secId, "robotWidth", 6665);			
	robotMass =            cf->ReadFloat (secId, "robotMass", 6665);						
	robotMI =              cf->ReadFloat (secId, "robotMI", 6665);									
	cnt =	 		   	   cf->GetTupleCount(secId,"robotCenter");
	if (cnt != 2)
	{
		cout<<"\n ERROR: center should consist of 2 tuples !!!";
		exit(1);
	}
	
	robotCenter.setX(cf->ReadTupleFloat(secId,"robotCenter",0 ,0));
	robotCenter.setY(cf->ReadTupleFloat(secId,"robotCenter",1 ,0));			
	findR();
    qDebug("-> Robot Configurations Read."); 
    qDebug("*********************************************************************"); 	
   	qDebug("\t\t Robot  name:\t%s", qPrintable(robotName)); 
    qDebug("\t\t Robot Ip is:\t%s:%d", qPrintable(robotIp),robotPort);
//   	qDebug("\t\t Robot Radius:%f",robotRadius); 
    qDebug("*********************************************************************");
  	return 1;
}

/*! This Determines the locations of the points to be checked in the Vehicle Coordinates,
 * should be rotated at each node */
void Robot::setCheckPoints(double obst_r) 
{
	this->obstacleRadius = obst_r;
	int point_index=0,points_per_height,points_per_width,n;
	double i,j, l = robotLength , w = robotWidth;
	check_points.clear();
	// The edges of the robot in -ve quadrant
	double startx,starty,internal_radius;       
	QPointF temp,edges[4];   
	edges[0].setX(-l/2);		edges[0].setY(w/2);
	edges[1].setX(l/2);			edges[1].setY(w/2);
	edges[2].setX(l/2);			edges[2].setY(-w/2);
	edges[3].setX(-l/2);		edges[3].setY(-w/2);	
	Pose pose(-robotCenter.x(),-robotCenter.y(),0);
//	cout <<"\nCenter Point X:"<<center.x()<<" Y:"<<center.y();		
	// am determining here the location of the edges in the robot coordinate system
	startx = -l/2 - robotCenter.x(); 
	starty = -w/2 - robotCenter.y();
	// These Points are used for drawing the Robot rectangle
	for(int s=0;s<4;s++)
	{
		local_edge_points.push_back(Trans2Global(edges[s],pose));
	}
//	for (int s=0 ;s < 4; s++)
//		cout<<"\nEdge->"<< s<<" X="<<local_edge_points[s].x()<<" Y="<<local_edge_points[s].y();
	// Create a Matrix of the points to check for collision detection
	internal_radius = this->obstacleRadius/sqrt(2);
	points_per_height = (int)(ceil(l/(double)(2*internal_radius)));
	points_per_width  = (int)(ceil(w/(double)(2*internal_radius)));
	n = points_per_height*points_per_width;
//	cout<<"\nPer H ="<<points_per_height<<" Per W="<<points_per_width<<" Total ="<<n;
//	cout<<"\n Obstacle Radius="<<internal_radius; fflush(stdout);

	// The location of the current edges at each NODE
	i =(startx + internal_radius);
	for(int r =0; r < points_per_height ; r++ )
	{
		j=(starty + internal_radius);
		for (int s=0;s < points_per_width;s++)
		{
			// Angle zero is when robot heading points to the right (right had rule)
			temp.setX(i);
			temp.setY(j);
			check_points.push_back(temp);
			point_index++;
			//cout<<"\n I="<<i<<" J="<<j;
			/* Determining the next center it should be 2*r away from the previous
			 * and it's circle should not exceed the boundaries
			 */
			if ( (j+2*internal_radius + internal_radius) >= (w + starty) ) 
				j = (w + starty - internal_radius);// Allow overlap
			else 
				j += (2*internal_radius);
		}
		// Same as Above
//		if ((i+2*internal_radius + internal_radius) >= (l + startx)) 
//		{
//			// Alow overlap in this case, this is the last center
//			i = (l + startx - internal_radius); 
//		}
//		else 
			i += (2*internal_radius);
	}
	for (int k=0;k<check_points.size();k++)
	{
		cout << "\nPoint to check "<<k<<"'---> X="<<check_points[k].x()<<" Y="<<check_points[k].y();
		fflush(stdout);
	}
	findR();
};
void Robot::findR()
{
	double dist,max_dist=-10;
 	for (int i = 0; i < 4; i++)
 	{
// 		dist = Dist(robotCenter,local_edge_points[i]);
 		dist = Dist(QPointF(0,0),local_edge_points[i]);
 		if (dist > max_dist)
 			max_dist = dist;
 	}
	this->robotRadius= max_dist;
}	

void Robot::setPose(Pose location)
{
	this->robotLocation = location;
}

Robot::Robot() 
{
};
Robot::~Robot() 
{
};

#include "Robot.h"

namespace CasPlanner
{
/* This Determines the locations of the points to be checked in the Vehicle Coordinates,
 * should be rotated at each node */
void Robot::SetCheckPoints() 
{
	int point_index=0,points_per_height,points_per_width,n;
	double i,j, l = length , w = width;
	check_points.clear();
	// The edges of the robot in -ve quadrant
	double startx,starty;       
	QPointF temp,edges[4];   
	edges[0].setX(-l/2);		edges[0].setY(w/2);
	edges[1].setX(l/2);			edges[1].setY(w/2);
	edges[2].setX(l/2);			edges[2].setY(-w/2);
	edges[3].setX(-l/2);		edges[3].setY(-w/2);	
	Pose pose(-center.x(),-center.y(),0);
//	cout <<"\nCenter Point X:"<<center.x()<<" Y:"<<center.y();		
	// am determining here the location of the edges in the robot coordinate system
	startx = -l/2 - center.x(); 
	starty = -w/2 - center.y();
	// These Points are used for drawing the Robot rectangle
	for(int s=0;s<4;s++)
	{
		local_edge_points[s] = Trans2Global(edges[s],pose)		;
	}
//	for (int s=0 ;s < 4; s++)
//		cout<<"\nEdge->"<< s<<" X="<<local_edge_points[s].x()<<" Y="<<local_edge_points[s].y();
	// Create a Matrix of the points to check for collision detection
	points_per_height = (int)(ceil(l/(double)(2*this->obstacle_radius)));
	points_per_width  = (int)(ceil(w/(double)(2*this->obstacle_radius)));
	n = points_per_height*points_per_width;
//	cout<<"\nPer H ="<<points_per_height<<" Per W="<<points_per_width<<" Total ="<<n;
//	cout<<"\n Obstacle Radius="<<this->obstacle_radius; fflush(stdout);

	// The location of the current edges at each NODE
	i =(startx + this->obstacle_radius);
	for(int r =0; r < points_per_height ; r++ )
	{
		j=(starty + this->obstacle_radius);
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
			if ( (j+2*this->obstacle_radius + this->obstacle_radius) >= (w + starty) ) 
				j = (w + starty - this->obstacle_radius);// Allow overlap
			else 
				j += (2*this->obstacle_radius);
		}
		// Same as Above
		if ((i+2*this->obstacle_radius + this->obstacle_radius) >= (l + startx)) 
		{
			// Alow overlap in this case, this is the last center
			i = (l + startx - this->obstacle_radius); 
		}
		else 
			i += (2*this->obstacle_radius);
	}
	for (unsigned int k=0;k<check_points.size();k++)
	{
		cout << "\nPoint to check "<<k<<"'---> X="<<check_points[k].x()<<" Y="<<check_points[k].y();
		fflush(stdout);
	}
};
Robot::Robot (double l, double w,double o_r,QString model,QPointF r_c):
	length(l),
	width(w),
	obstacle_radius(o_r),	
	model(model),
	center(r_c)
{
	SetCheckPoints();
};

Robot::Robot() 
{
};
Robot::~Robot() 
{
};
}

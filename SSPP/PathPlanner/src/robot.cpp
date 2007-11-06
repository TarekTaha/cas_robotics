/***************************************************************************
 *   Copyright (C) 2007 by Tarek Taha                                      *
 *   tataha@eng.uts.edu.au                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "robot.h"
namespace CasPlanner
{
//! Consturctor for the Robot Class
/*! This constructor take the following Parameters:
 * @param name is a string variable representing the Robot's Name.
 * @param length is a double variable representing the Length of the Robot.
 * @param width is a double variable representing the width of the Robot.
 * @param narrowDist is a double variable representing the narrowest distance in the environment. This distance is used 
 * to reduce the collision detection between the Robot and the surroundings into a set of points.
 * @param center is a #Point variable representing the position of the center of rotation of the Robot with respect
 * to the center of Area. This is also used for collision detection.
 */
Robot::Robot(string name,double length,double width,double narrowDist,Point center):
robotLength(length),
robotWidth(width),
narrowestPathDist(narrowDist),
robotName(name),
robotCenter(center)
{
	this->setCheckPoints(narrowDist);
}

/*! This Determines the locations of the points to be checked in the Vehicle Coordinates,
 * should be rotated at each node 
 */
void Robot::setCheckPoints(double obst_r)
{
	/*! 
	 * Based on our environment, the narrowest passage is X pixels of width
	 * and this should be taken into consideration while expanding and checking
	 * for collision. check my "An Efficient Path Planner for Large Mobile Platforms" paper
	 * for more information
	 */ 
	safetyTolerance = 0.05;
	if(narrowestPathDist < 2*robotWidth)
	{
		expansionRadius = (narrowestPathDist - safetyTolerance)/2.0f;
	}
	else
		expansionRadius = robotWidth/2.0f - safetyTolerance;
	cout<<"\nObstacle Expansion Radius="<<expansionRadius;
	int point_index=0,points_per_height,points_per_width,n;
	double i,j, l = robotLength , w = robotWidth;
	check_points.clear();
	// The edges of the robot in -ve quadrant
	Point temp,edges[4];
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
	for (int s=0 ;s < 4; s++)
		cout<<"\nEdge->"<< s<<" X="<<local_edge_points[s].x()<<" Y="<<local_edge_points[s].y();
	// Create a Matrix of the points to check for collision detection

	points_per_height = 2;//(int)(ceil(l/(double)(2*expansionRadius)));
	points_per_width  = (int)(ceil(w/(double)(2*expansionRadius)));
	n = points_per_height*points_per_width;
	cout<<"\nPer H ="<<points_per_height<<" Per W="<<points_per_width<<" Total ="<<n;

	// The location of the current edges at each NODE
	i = startx + sqrt(expansionRadius*expansionRadius - pow(w/2.0f,2) - safetyTolerance);
	for(int r =0; r < points_per_height ; r++ )
	{
		j=(starty + expansionRadius);
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
			if ( (j+2*expansionRadius + expansionRadius) >= (w + starty) )
				j = (w + starty - expansionRadius);// Allow overlap
			else
				j += (2*expansionRadius);
		}
		// Same as Above
		if ((i+2*expansionRadius + expansionRadius) >= (l + startx))
		{
			// Alow overlap in this case, this is the last center
			i = (l + startx - expansionRadius);
			i = l + startx - sqrt(expansionRadius*expansionRadius - pow(w/2.0f,2) - safetyTolerance);
		}
		else
			i += (2*expansionRadius);
	}
	for (unsigned int k=0;k<check_points.size();k++)
	{
		check_points[k].setY(0);
		cout << "\nPoint to check "<<k<<"'---> X="<<check_points[k].x()<<" Y="<<check_points[k].y();
		fflush(stdout);
	}
};
//! Accessor method to the expansion Radius
double Robot::getExpansionRadius()
{
	return this->expansionRadius;
}
//! Robot's destructor.
Robot::~Robot()
{
};

}

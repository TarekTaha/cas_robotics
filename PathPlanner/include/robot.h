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
#ifndef ROBOT_H_
#define ROBOT_H_

#include <common.h>
#include <string>
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;
namespace CasPlanner
{
class Robot
{
	private:
		void setCheckPoints(double o_r);
		double robotLength,robotWidth,obstacleRadius,robotRadius, expansionRadius,narrowestPathDist,safetyTolerance, startx,starty;
		//! Motion model
		string robotModel,robotName;
		// Center of Rotation
		Point robotCenter;
	public :
		//! Points used to render the Robot's Rectangle.
		vector<Point> local_edge_points;
		//! Points to be used for Collision Checks
		vector<Point> check_points;
		double getExpansionRadius();
		Robot(string name,double length,double width,double narrowDist,Point center);
		~Robot();
};
}
#endif /*ROBOT_H_*/

#include "Robot.h"

namespace CasPlanner
{

void Robot::SetCheckPoints(int number_of_points,Point * a) 
{
	//this->check_points = new Point[number_of_points];
	for(int i = 0; i < number_of_points; i++)
	{
		check_points.push_back(a[i]);
		//cout << "\n New Robot i="<<i<<" X=" <<this->check_points[i].x<<" Y="<<this->check_points[i].y;
	}
};
Robot::Robot() 
	{
	};
Robot::~Robot() 
	{
	//delete [] this->check_points;
	//cout << "\n Robot Freed ";
	};
}

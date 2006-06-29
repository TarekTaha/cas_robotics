#include "Robot.h"

namespace CasPlanner
{

void Robot::SetCheckPoints(vector<QPointF> a) 
{
	for(int i = 0; i < a.size(); i++)
	{
		check_points.push_back(a[i]);
		//cout << "\n New Robot i="<<i<<" X=" <<this->check_points[i].x<<" Y="<<this->check_points[i].y;
	}
};

Robot::Robot (double l, double w,QString model )
{
	this->width = w;
	this->length = l;
	this->model = model;
};

Robot::Robot() 
	{
	};
Robot::~Robot() 
	{
	};
}

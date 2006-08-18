#ifndef ROBOT_H_
#define ROBOT_H_
#include <QPointF>
#include <QString>
#include <vector>
#include <iostream>
#include <math.h>
#include "utils.h"
using namespace std;
using std::vector;
namespace CasPlanner
{

class Robot
{
	public :
		double length,width,obstacle_radius,robotMass,robotMI,robotRadius;
		double TimeStep, SysK, SysC, SysFR, SysP, SysQ, MaxSpeed, MaxAcceT, OmegadotMax, OmegaMax;
		// Motion model
		QString model;
		// Center of Rotation
		QPointF center;
		// For Rendering the Robot Rectangle
		QPointF local_edge_points[4];
		vector<QPointF> check_points;
		void SetCheckPoints();
		Robot (double r_l, double r_w,double o_r,QString r_m,QPointF r_c );
		Robot();
		~Robot();
};
}

#endif /*ROBOT_H_*/

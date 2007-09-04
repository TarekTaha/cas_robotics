#ifndef ROBOT_H_
#define ROBOT_H_
#include <QPointF>
#include <QString>
#include <QVector>
#include <iostream>
#include <math.h>
#include "utils.h"
#include "configfile.h"

using namespace std;
class Robot
{
	public :
		double robotLength,robotWidth,obstacleRadius,robotMass,robotMI,robotRadius,
			   robotSpeed,robotTurnRate,expansionRadius,narrowestPathDist,safetyTolerance, startx,starty;			   
		int robotPort;
		//! Motion model
		QString robotModel,robotName,robotIp;
		//! Holds the Latest Robot Position
		Pose robotLocation;
		// Center of Rotation
		QPointF robotCenter;
		// For Rendering the Robot Rectangle
		QVector<QPointF> local_edge_points, check_points;
		void setCheckPoints(double o_r);
		void setPose(Pose location);
		void setSpeed(double speed);
		void setTurnRate(double turnRate);
		void findR();
		int  readConfigs(ConfigFile *cf,int secId);
		Robot(ConfigFile *cf,int secId);
		Robot();
		~Robot();
};

#endif /*ROBOT_H_*/

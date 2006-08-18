#ifndef FORCEFIELD_H_
#define FORCEFIELD_H_
#include<Robot.h>
#include<QPointF>
#include<QVector>
#include<math.h>
#include "utils.h"
#include "robot.h"
using std::vector;

namespace CasPlanner
{
class ForceField : public Robot
{
public:
	ForceField();
	virtual ~ForceField();
	VelVector ForceField::GenerateField(QPointF position,QVector<QPointF> laser_set,QPointF Goal,QPointF location,double speed,double turnrate);
private :
	double NearestObstacle(QVector<QPointF> laser_scan,Pose lase_pose);
	double DotMultiply(POINT, POINT, POINT);
	const int curvefittingorder = 6;
	const double Gapdist = 0.5;
	const int NPOL = 4;
	const double INF = 1E200;
	const double EP = 1E-10;
	double robotSpeed,robotTurnRate;
	Pose robotLocation, goalLocation;
	QVector <QPointF> laser_readings;
};
}
#endif /*FORCEFIELD_H_*/

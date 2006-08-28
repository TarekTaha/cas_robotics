#ifndef FORCEFIELD_H_
#define FORCEFIELD_H_
#include <Robot.h>
#include <QPointF>
#include <QVector>
#include <math.h>

#include "utils.h"
#include "Robot.h"
#include "Vector2D.h"

using std::vector;
typedef struct _interaction
	{
		QPointF location;
		double force;
		double direction;
	}   Interaction;
typedef struct _velVector
{
	double speed;
	double turnRate;
}   velVector;	
class ForceField : public Robot
{
public:
	ForceField(Robot r,ConfigFile *cf);
	virtual ~ForceField();
	velVector GenerateField(Pose pose,QVector<QPointF> laser_set,Pose Goal,double speed,double turnrate);
	QVector < QVector<QPointF> > DivObst(QVector<QPointF> laser_set);	
	void LSCurveFitting (QVector<QPointF> obstacle, double a[], int m);
	double Dist2Robot(QPointF ray_end,Pose laser_pose,double &angle);
	void VSFF(QVector<Interaction> obstacle_interaction_set);	
	double FindNorm(QPointF interaction_point, double Tang);
	void CrossProduct(double MatrixA[3], double MatrixB[3], double MatrixC[3]);
	double ForceValue(QPointF ray_end,Pose laser_pose);
private :
	double   FixedRatio,TimeStep,SysK,SysC,SysP,SysQ,MaxSpeed,MaxAcceT,OmegadotMax,OmegaMax,
			 Gapdist,NPOL,INF,EP;
	int curvefittingorder;
	double robotSpeed,robotTurnRate;
	QPointF intersect_point,end_point;
	Pose robotLocation, goalLocation;
	QVector <QPointF> laser_readings;
};
#endif /*FORCEFIELD_H_*/

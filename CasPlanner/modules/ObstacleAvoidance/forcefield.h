/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#ifndef FORCEFIELD_H_
#define FORCEFIELD_H_
#include <QPointF>
#include <QVector>
#include <math.h>
#include "utils.h"
#include "robot.h"
#include "Vector2D.h"
#include "playerinterface.h"

using std::vector;
typedef struct _interaction
	{
		QPointF location;
		double force;
		double direction;
		double closestdist;
		double angle;
	}   Interaction;
typedef struct _velVector
{
	double speed;
	double turnRate;
}   velVector;

class ForceField : public Robot
{
	enum {VariableSpeedFF,SimpleFF};
public:
	ForceField(Robot r,ConfigFile *cf);
	virtual ~ForceField();
    velVector GenerateField(Pose pose,LaserScan laser_set,Pose Goal,double speed,double turnrate,QVector <Robot*> robots,double deltaTime);
	QVector < QVector<QPointF> > DivObst(LaserScan laser_set,Pose laser_pose);	
	void    LSCurveFitting (QVector<QPointF>, double[], int);
	double  Dist2Robot(QPointF,double &);
	void    VSFF(QVector<Interaction>, QVector<Interaction>, double);	
	double  FindNorm(QPointF interaction_point, double Tang);
	void    CrossProduct(double MatrixA[3], double MatrixB[3], double MatrixC[3]);
	//void    SimFF(QVector<Interaction> obstacle_interaction_set);
	double  ForceValue(QPointF ray_end, double &, double &, double &, double &);
	double  Delta_Angle(double, double);
	void    robotForceFieldShape(Robot*, QVector<QPointF>&, QVector<QPointF>&);
	//double  ForceValue_dynamicobstalce(dymamicobstacle, obstacleDmaxPoint);
	void    SimFF(QVector<Interaction>, QVector<Interaction>, double);
	QVector<Interaction> getDynamicInteractionSet(QVector <Robot*> robots);
	//double  ForceValue_dynamicobstalce(dymamicobstacle, obstacleDmaxPoint);
private :
	double   FixedRatio,TimeStep,SysK,SysC,SysP,SysQ,MaxSpeed,MaxAcceT,OmegadotMax,OmegaMax,
			 Gapdist,NPOL,INF,EP;
	int curvefittingorder,FF_algorithm;
	double robotSpeed,robotTurnRate;
	QPointF intersect_point,end_point;
	Pose robotLocation, goalLocation;
	QVector <QPointF> laser_readings;
};
#endif /*FORCEFIELD_H_*/

#ifndef FORCEFIELD_H_
#define FORCEFIELD_H_
#include<Robot.h>
#include<QPointF>
#include<QVector>
using std::vector;
namespace CasPlanner
{
class ForceField
{
public:
	ForceField();
	ForceField(Robot * robot);
	virtual ~ForceField();
	void GenerateField(QPointF position,QVector<QPointF> laser_set,QPointF Goal);
private :
	double robot_v, robot_w,
	       cntrl_v, cntrl_w;
	QVector <QPointF> laser_readings;
};
}
#endif /*FORCEFIELD_H_*/

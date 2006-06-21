#ifndef FORCEFIELD_H_
#define FORCEFIELD_H_
#include<Robot.h>
#include<Point.h>
#include<vector>
using std::vector;
namespace CasPlanner
{
class ForceField
{
public:
	ForceField();
	ForceField(Robot * robot);
	virtual ~ForceField();
	void GenerateField(Point position,vector<Point> laser_set,Point Goal);
private :
	double robot_v, robot_w,
	       cntrl_v, cntrl_w;
	vector <Point> laser_readings;
};
}
#endif /*FORCEFIELD_H_*/

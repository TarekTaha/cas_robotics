#ifndef FORCEFIELD_H_
#define FORCEFIELD_H_

class ForceField
{
public:
	ForceField();
	ForceField(Robot * robot);
	virtual ~ForceField();
	void GenerateField(Point position,vector<Point> laser_set);
private :
	double robot_v, robot_w;
	       cntrl_v, cntrl_w;
	vector <Point> laser_readings;
	PlayerClient 	*robot;
	PositionProxy 	*pp;
	LaserProxy 		*laser;
	LocalizeProxy 	*localizer; ;;
};

#endif /*FORCEFIELD_H_*/

#ifndef ROBOT_H_
#define ROBOT_H_
#include<Point.h>
#include<vector>
using std::vector;
namespace CasPlanner
{

class Robot
	{
	public :
		//Point * check_points;
		double length,width;
		vector<Point> check_points;
		void SetCheckPoints(int,Point *);
		Robot();
		~Robot();	
	};
}

#endif /*ROBOT_H_*/

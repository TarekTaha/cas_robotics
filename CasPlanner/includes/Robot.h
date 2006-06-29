#ifndef ROBOT_H_
#define ROBOT_H_
#include <QPointF>
#include <QString>
#include <vector>
using std::vector;
namespace CasPlanner
{

class Robot
	{
	public :
		double length,width;
		// Motion model
		QString model;
		vector<QPointF> check_points;
		void SetCheckPoints(vector<QPointF>);
		Robot (double l, double w,QString model )
		Robot();
		~Robot();	
	};
}

#endif /*ROBOT_H_*/

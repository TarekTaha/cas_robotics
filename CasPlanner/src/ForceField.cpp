#include "ForceField.h"
//Empty constructor
namespace CasPlanner
{
	ForceField::ForceField()
	{
	}
	// Constructor with the Robot Size
	ForceField::ForceField(Robot * robot)
	{
	}
	// Destructor -> Free Memory
	ForceField::~ForceField()
	{
	}
	/* Given a Position and an environment, generate the action to 
	 * go to the goal Point without colliding with the surrondings
	 */
	void ForceField::GenerateField(QPointF position,QVector<QPointF> laser_set,QPointF Goal)
	{
		//position.x;
		//position.y;
		//for(i=0;i<laser_set.size();i++)
		{
			
		}
	}
};

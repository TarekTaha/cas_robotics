#include "ForceField.h"
//Empty constructor
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
void ForceField::GenerateField(Point position,vector<Point> laser_set,Point Goal)
{
	//position.x;
	//position.y;
	for(i=0;i<laser_set.size();i++)
	{
		
	}
}
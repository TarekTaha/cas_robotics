#ifndef NODE_H_
#define NODE_H_
#include<Point.h>
#include<Robot.h>
namespace CasPlanner
{

class Node
	{
	public :
		int id, depth,direction;
		double nearest_obstacle,g_value,h_value,f_value,angle;
		Node  * parent, * next, * prev;
		Point   location;
		// Saving the location of the Robot edges on the planned path
		Robot  wheelchair;
		Node ();
		bool operator == (Node);
		~Node();	
	};

}

#endif /*NODE_H_*/

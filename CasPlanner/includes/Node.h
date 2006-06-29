#ifndef NODE_H_
#define NODE_H_
#include<QPointF>
#include<Robot.h>
namespace CasPlanner
{
/* This Class Represents the Node Structure in the search Tree,
 * each node encapsulates information about it's parent, next
 * and previous node in the list, it's location, travelling and
 * herustic costs.
 */
class Node
	{
	public :
		int id,depth,direction;
		double nearest_obstacle,g_value,h_value,f_value,angle;
		Node  * parent, * next, * prev;
		QPointF   location;
		// Saving the location of the Robot edges on the planned path
		Node ();
		bool operator == (Node);
		~Node();	
	};

}

#endif /*NODE_H_*/

#ifndef NODE_H_
#define NODE_H_

#include<QPointF>
#include "Robot.h"
#include "utils.h"

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
		double nearest_obstacle,g_value,h_value,f_value;
		Node  * parent, * next, * prev;
		Pose   pose;
		Node ();
		bool operator == (Node);
		bool operator != (Node);		
		~Node();	
	};

}

#endif /*NODE_H_*/

#include "Node.h"

namespace CasPlanner
{

Node :: Node ()
	{
		parent = next = prev = NULL;
	};
Node :: ~Node ()
	{
		parent = next = prev = NULL;
	};
bool Node ::operator == (Node a)
	{
	if (this->location.x == a.location.x && this->location.y == a.location.y)
		return 1;
	return 0;		
	}

}

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
	return (this->pose.p.x() == a.pose.p.x() && this->pose.p.y() == a.pose.p.y());
}

}

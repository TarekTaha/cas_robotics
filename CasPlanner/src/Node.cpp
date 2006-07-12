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
	return ( isEqual(this->pose.p.x(),a.pose.p.x()) && isEqual(this->pose.p.y(),a.pose.p.y()));
}

}

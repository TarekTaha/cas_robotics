#ifndef SEARCHSPACE_H_
#define SEARCHSPACE_H_

#include "searchspacenode.h"
namespace CasPlanner
{

class SearchSpace
{
public:
	SearchSpaceNode * search_space;
	void freeSearchSpace();
	SearchSpaceNode * insertNode(QPointF loc);
	SearchSpaceNode * nodeExists(QPointF loc);
	bool              removeNode(QPointF loc);
	SearchSpace();
	virtual ~SearchSpace();
};

}

#endif /*SEARCHSPACE_H_*/

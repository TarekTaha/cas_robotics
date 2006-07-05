#ifndef SEARCHSPACE_H_
#define SEARCHSPACE_H_

#include "SearchSpaceNode.h"
namespace CasPlanner
{

class SearchSpace
{
public:
	SearchSpaceNode *search_space;
	SearchSpace();
	virtual ~SearchSpace();
};

}

#endif /*SEARCHSPACE_H_*/

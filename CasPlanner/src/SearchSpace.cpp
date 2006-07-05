#include "SearchSpace.h"

namespace CasPlanner
{

SearchSpace::SearchSpace():
search_space(NULL)
{
}

SearchSpace::~SearchSpace()
{
	SearchSpaceNode *temp;
	while (search_space != NULL)
	{
		temp = search_space;
		search_space = search_space->next;
		delete temp;
	};
}

void SearchSpace:: FreeSearchSpace()
{
	SearchSpaceNode *temp;
	while (search_space != NULL)
	{
		temp = search_space;
		search_space = search_space->next;
		delete temp;
	};
}

}

#include "searchspace.h"

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

SearchSpaceNode * SearchSpace::insertNode(QPointF loc)
{
  	SearchSpaceNode *temp;	
  	if(nodeExists(loc))
  		return NULL;
	if (search_space == NULL ) // Constructing the ROOT NODE
	{
		temp = new SearchSpaceNode;
		temp->location.setX(loc.x());
		temp->location.setY(loc.y());
		temp->obstacle_cost = 0;
		temp->parent   = NULL;
		temp->next     = NULL;
		search_space = temp;
	}
	else
	{
		temp = new SearchSpaceNode;
		temp->location.setX(loc.x());
		temp->location.setY(loc.y());
		temp->obstacle_cost = 0;
		temp->parent = NULL; 
		temp->next   = search_space;
		search_space = temp;
	}	
	return temp;
}

SearchSpaceNode * SearchSpace::nodeExists(QPointF loc)
{
	SearchSpaceNode *temp = search_space;
	while (temp != NULL)
	{
		if(loc == temp->location)
			return temp;
		temp = temp->next;
	}
	// node with the given location does not exist
	return NULL;
}

bool SearchSpace::removeNode(QPointF loc)
{
	SearchSpaceNode *temp = search_space;
	while (temp != NULL)
	{
		if(loc == temp->location)
		{
			temp->parent->next = temp->next;
			delete temp;
			return true;
		}
		temp = temp->next;
	}
	// Not found in the list
	return false;	
}

void SearchSpace:: freeSearchSpace()
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

/***************************************************************************
 *   Copyright (C) 2007 by Tarek Taha                                      *
 *   tataha@eng.uts.edu.au                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "searchspace.h"

namespace CasPlanner
{
//! Constructor of the SearchSpace
SearchSpace::SearchSpace():
searchSpace(NULL)
{
}
//! Destructor of the SearchSpace
SearchSpace::~SearchSpace()
{
	SearchSpaceNode *temp;
	while (searchSpace != NULL)
	{
		temp = searchSpace;
		searchSpace = searchSpace->next;
		delete temp;
	};
}
//! Inserts a Node into the appropriate location
SearchSpaceNode * SearchSpace::insertNode(Point loc)
{
  	SearchSpaceNode *temp;	
  	if(nodeExists(loc))
  		return NULL;
	if (searchSpace == NULL ) // Constructing the ROOT NODE
	{
		temp = new SearchSpaceNode;
		temp->location.setX(loc.x());
		temp->location.setY(loc.y());
		temp->obstacle_cost = 0;
		temp->parent   = NULL;
		temp->next     = NULL;
		searchSpace = temp;
	}
	else
	{
		temp = new SearchSpaceNode;
		temp->location.setX(loc.x());
		temp->location.setY(loc.y());
		temp->obstacle_cost = 0;
		temp->parent = NULL; 
		temp->next   = searchSpace;
		searchSpace = temp;
	}	
	return temp;
}
//!  Check if a Node with the requesting location already exists in the Search Space
SearchSpaceNode * SearchSpace::nodeExists(Point loc)
{
	SearchSpaceNode *temp = searchSpace;
	while (temp != NULL)
	{
		if(loc == temp->location)
			return temp;
		temp = temp->next;
	}
	// node with the given location does not exist
	return NULL;
}
//! Removes a Node with the given location from the searchSpace
bool SearchSpace::removeNode(Point loc)
{
	SearchSpaceNode *temp = searchSpace;
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
//! Frees the allocated memeory Resources.
void SearchSpace:: freeSearchSpace()
{
	SearchSpaceNode *temp;
	while (searchSpace != NULL)
	{
		temp = searchSpace;
		searchSpace = searchSpace->next;
		delete temp;
	};
}

}

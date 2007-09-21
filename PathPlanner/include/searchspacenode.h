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
#ifndef SEARCHSPACENODE_H_
#define SEARCHSPACENODE_H_

#include<vector>
#include "common.h"

using namespace std;

enum {BridgeNode,RegGridNode};

namespace CasPlanner
{

class SearchSpaceNode
	{
	public :
		//! The location of this SearchSpace Node.
		Point location;
		//! Pointer to the Parent SearchSpace Node. 
		SearchSpaceNode  * parent;
		//! Pointer to the next SearchSpace Node.
		SearchSpaceNode * next;
		//! The type of this SearchSpace Node (BridgeNode or RegGridNode).
		int type;
		//! The obstacle penality cost added to this SearchSpace Node.
		double obstacle_cost;
		//! The children Nodes of this Node.
		vector <SearchSpaceNode *>  children;
		 SearchSpaceNode ();
		~SearchSpaceNode ();
	};
}

#endif /*SEARCHSPACENODE_H_*/

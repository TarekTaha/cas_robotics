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
#ifndef NODE_H_
#define NODE_H_

#include "common.h"
#include "robot.h"

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
		//! The id of this Node.
		int id;
		//! The Search Depth of this Node.
		int depth;
		//! The Direction of motion at this Node.
		int direction;
		//! The distance to the Nearest Obstacle.
		double nearest_obstacle;
		//! The herustic G Cost.
		double g_value;
		//! The herustic H Cost.
		double h_value;
		//! The herustic F Cost.
		double f_value;
		//! A Link to this Node's Parent.
		Node  * parent;
		//! A Link to the next Node.
		Node  * next;
		//! A Link to the previous Node.
		Node  * prev;
		//! The Post of this Node.
		Pose   pose;
		//! Constructor
		Node ();
		//! == Operator override. 
		bool operator == (Node);
		//! != Operator override.
		bool operator != (Node);		
		~Node();	
	};
}

#endif /*NODE_H_*/

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
#ifndef ASTAR_H_
#define ASTAR_H_

#include <math.h>
#include "llist.h"
#include "node.h"
#include "robot.h"
#include "searchspace.h"
#include "map.h"
#include "common.h"

namespace CasPlanner
{
//! This structure Hold the expanded tree while searching for a path and is used for debugging reasons ONLY!!
typedef struct _tree
	{
		//! The location of the current Branch.
		Point location;
		//! The list of sub Children for this Branch.
		std::vector <Point> children;
	}   Tree;
	
enum{METRIC,PIXEL};
	
class Astar: public SearchSpace
{
	private:
		void   findRoot();
		double gCost(Node *n);
		double hCost(Node *n);
		Node  *makeChildrenNodes(Node *parent) ;
		LList *openList,*closedList;
		vector <Tree> tree;
	public:
		//! Maximium number of node to expand before stopping search.
		long int MAXNODES;
		//! The acceptable distance from the goal that terminates the search.
		double	distGoal;
		//! The map/map grid to be used for the grid search + search space generation.
		Map    * map;
		//! The start position of the robot
		Pose start;
		//! The end position from the robot
		Pose end;
		//! The robot structure that is necessary for collision checks.
		Robot *robot;
		//! Root Node 
		Node *root;
		//! Current Node
		Node *current;
		//! The current Child's List
		Node *childList;
		//! The current Child
		Node *curChild;
		//! The path found
		Node *path;
		Astar(Robot *,double dG);
		Astar();
		//! frees an allocated node
		void   freeNode      (Node *);
		bool   inObstacle    (Point p, double angle);
		bool   goalReached   (Node *n);
		Node*  startSearch   (Pose start,Pose end,int);
		//! class Distructor
		virtual ~Astar();
};

}

#endif /*ASTAR_H_*/

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
#include "astar.h"

namespace CasPlanner
{
//! Constructor for the AStar Class.
Astar::Astar(Robot *rob,double dG):
	distGoal(dG),
	map(NULL),
	robot(rob),		
	root(NULL),
	path(NULL)
{
	openList   = new LList;
	closedList = new LList; 
}
//! Empty Constructor for the AStar Class.
Astar::Astar():
	distGoal(0.01),
	map(NULL),
	root(NULL),
	path(NULL)
{
	openList   = new LList;
	closedList = new LList; 
}

Astar::~Astar()
{
}

//! Tests whether the robot at this Pose collids.
/*! This method checks for collision between the Robot and the surroundings in the map.
 * This is done by checking if the #CasPlanner::Robot::check_points produced by the #CasPlanner::Robot class are inside 
 * obstacles or not.
 */
bool Astar :: inObstacle(Point P, double theta)
{
	int m,n;
	Point det_point;
	// Rotates and Translates the check points according to the vehicle position and orientation
	for (unsigned int i=0;i<robot->check_points.size();i++)
	{
		det_point.setX((robot->check_points[i].x()*cos(theta) - robot->check_points[i].y()*sin(theta) + P.x()));
		det_point.setY((robot->check_points[i].x()*sin(theta) + robot->check_points[i].y()*cos(theta) + P.y()));
		
		map->convert2Pix(&det_point);
		m = (int)(round(det_point.x()));
		n = (int)(round(det_point.y()));
		if (m <= 0 || n <= 0 || m >=map->getWidth() || n >=this->map->getHeight())
			return true;
		if (this->map->grid[m][n])
			return true;
	}
	return false;
};
//! Find the nearest node to the start
/*! This method find the closest #SearchSpaceNode to the robot's
 * starting location. This is done because the space here is discratized
 * and the robot's starting location is in the continuous space.
 */
void Astar::findRoot() 
{
	SearchSpaceNode * temp;
	if(!this->searchSpace)
		return;
	double distance,shortest_distance = 100000;
	// allocate and setup the root node
	root = new Node;	
	temp = this->searchSpace;
	while(temp!=NULL)
	{
		distance = Dist(temp->location,start.p);
		// Initialize the root node information and put it in the open list
		if (distance < shortest_distance) 
		{
			shortest_distance = distance;
			root->pose.p.setX(temp->location.x());
			root->pose.p.setY(temp->location.y());
		}
		temp = temp->next;
	}
	root->parent = NULL;
	root->next = NULL;
	root->prev = NULL;
	root->g_value = 0;;
	root->h_value = gCost(root);
	root->f_value = root->g_value + root->h_value;
	root->id = 0;
	root->depth = 0;
	root->pose.phi = start.phi;
	root->direction = FORWARD;
	//Translate(root->pose,start.phi);
	printf("\n	---->>>Root is Set to be X=%f Y=%f Phi=%f",root->pose.p.x(),root->pose.p.y(),RTOD(root->pose.phi));
};
//! Starts the search for Path from Start Position to the end Position
/*! This method starts the actual AStar search from the start to the end.
 * @param start is the start Pose.
 * @param end   is the end   Pose.
 * @param coord is the reference coordinate system to be used (PIXEL or METRIC).
 * @return The Path which is an list of Node specifying the segment ends. 
 */
Node *  Astar::startSearch(Pose start,Pose end, int coord)
{
	int      ID = 1;
  	int      NodesExpanded = 0;
	if(this->tree.size() > 0)		
		this->tree.clear();
	if(!openList)
	{
		openList   = new LList;
	}
	if(!closedList)
	{
		closedList = new LList; 			
	}
	// Be sure that open and closed lists are empty
	openList->Free();
	closedList->Free();	
	if (!this->searchSpace) // Make sure that we have a map to search
	{
		printf("\nRead the map and generate SearchSpace before Searching !!!");
		return NULL;
	}
	if(coord == PIXEL)
	{
		map->convertPix(&start.p);
		map->convertPix(&end.p);
	}
	this->start.p.setX(start.p.x());
	this->start.p.setY(start.p.y());
	this->start.phi = start.phi;
	this->end.p.setX(end.p.x());
	this->end.p.setY(end.p.y());
	this->end.phi = end.phi;
	printf("\n	--->>> Search Started <<<---");
	findRoot();
	printf("\n	---->>>Target is Set to be X=%f Y=%f Phi=%f<<<---",end.p.x(),end.p.y(),RTOD(end.phi));
  	openList->Add(root);				// Add the root to OpenList
	printf("\n	--->>> Root Added <<<---");
	
	// while openList is not empty 
	while (openList->Start != NULL) 
	{
		current = openList->GetHead(); 	// Get the node with the cheapest cost (first node)
		openList->Next();				// Move to the next Node
    	NodesExpanded++;
    	// We reached the target pose, so build the path and return it.
    	if (goalReached(current) && current!= root)                     
		{
			// build the complete path to return
			printf("\nLast Node destination: %f %f",current->pose.p.x(),current->pose.p.y());
      		current->next = NULL;
    		printf("\n	--->>> Goal state reached with :%d nodes created and :%d nodes expanded <<<---",ID,NodesExpanded);
			printf("\n	--->>> General Clean UP <<<---");
			fflush(stdout);
//			int m=0;
//	   		while (p != NULL) 
//				{
//					cout<<"\n	--->>> Step["<<++m<<"] X="<<p->pose.p.x()<<" Y="<<p->pose.p.y();
//					//cout<<"\n	--->>> Angle is="<<RTOD(p->angle);
//					fflush(stdout);
//					p = p->parent;
//				}
//			Going up to the Root
			Node * p;
      		p = current;
			path = NULL;
   			while (p != NULL)
			{
//				cout<<"\n Am Still HERE Step["<<++m<<"] X="<<p->pose.x<<" Y="<<p->pose.y;
//				fflush(stdout);
				// remove the parent node from the closed list (where it has to be)
				if(p->prev != NULL)
					(p->prev)->next = p->next;
				if(p->next != NULL)
					(p->next)->prev = p->prev;
				// check if we're removing the top of the list
				if(p == closedList->Start)
		  			closedList->Next();
				// set it up in the path
				p->next = path;				
				path = p;
				p = p->parent;
			}
        	// now delete all nodes on OPEN and Closed Lists
			printf("\n	--->>> Freeing open list <<<---");	fflush(stdout);
			openList->Free();
			printf("\n	--->>> DONE  <<<---");				fflush(stdout);
			printf("\n	--->>> Freeing closed list <<<---");	fflush(stdout);
			closedList->Free();
			printf("\n	--->>> DONE  <<<---");
      		return path; 	// Path Found Successfully
    	}
    	
    	// Create List of Children for the current NODE
		if(!(childList = makeChildrenNodes(current))) // No more Children => Search Ended Unsuccessfully at this path Branch
		{
			printf("\n	--->>> Search Ended On this Branch / We Reached a DEAD END <<<---");
		}
		// insert the children into the OPEN list according to their f values
    	while (childList != NULL)                     
		{
  		    curChild  = childList;
  			childList = childList->next;
		    // set up the rest of the child node details
  			curChild->parent = current;
  			curChild->depth  = current->depth + 1;
  			curChild->id = ID++;
  			//printf("ID is:%d",ID);
  			curChild->next = NULL;
  			curChild->prev = NULL;
  			curChild->g_value = gCost(curChild);
      		curChild->h_value = hCost(curChild);
  			curChild->f_value = curChild->g_value + curChild->h_value;
			Node * p;
			// check if the child is already in the open list
			if( (p = openList->Find(curChild)))
			{
  				if (p->f_value <= curChild->f_value && (p->direction == curChild->direction))       
				{
	        		freeNode(curChild);
  					curChild = NULL;
				}
				// the child is a shorter path to this point, delete p from  the closed list
				else 
  				if (p->f_value > curChild->f_value && (p->direction == curChild->direction))
				{
					openList->Remove(p);
				 	//cout<<"\n	--->>> Opened list -- Node is deleted, current child X="<<curChild->pose.x<<" Y="<<curChild->pose.y<<" has shorter path<<<---";
					fflush(stdout);

				}
			}
			// test whether the child is in the closed list (already been there)			
			if (curChild)
			{
				if((p = closedList->Find(curChild)))
				{
	  				if (p->f_value <= curChild->f_value && p->direction == curChild->direction)       
					{
		        		freeNode(curChild);
	  					curChild = NULL;
					}
					// the child is a shorter path to this point, delete p from  the closed list
					else 
					{
						/* This is the tricky part, it rarely happens, but in my case it happenes all the time :s
						 * Anyways, we are here cause we found a better path to a node that we already visited, we will have to
						 * Update the cost of that node and ALL ITS DESCENDENTS because their cost is parent dependent ;)
						 * Another Solution is simply to comment everything and do nothing, doing this, the child will be added to the
						 * Open List and it will be investigated further later on.
						 */
						//TODO : this SHOULD be fixed, very very DODGY
//						Node *ptr = closedList->Start;
//						while(ptr)
//						{
//							if(ptr->parent == p)
//								ptr->parent = NULL;
//							ptr = ptr->next;
//						}
//						closedList->Remove(p);
					 	//cout<<"\n	--->>> Closed list -- Node is deleted, current child X="<<curChild->pose.x<<" Y="<<curChild->pose.y<<" has shorter path<<<---";
						fflush(stdout);
	
					}					
				}
			}
			// ADD the child to the OPEN List
			if (curChild)
			{
				openList->Add(curChild);
			}	 		
    	}
		// put the current node onto the closed list, ==>> already visited List
		closedList->Add(current);
		// Test to see if we have expanded too many nodes without a solution
    	if (current->id > this->MAXNODES)     	
		{
	    	printf("\n	--->>>	Expanded %d Nodes which is more than the maximum allowed MAXNODE=%ld , Search Terminated",current->id,MAXNODES);
			//Delete Nodes in Open and Closed Lists
			closedList->Free();
			openList->Free();
			path = NULL;
			return path; // Expanded more than the maximium nodes state
    	}
 	}	//...  end of OPEN loop
 	
  	/* if we got here, then there is no path to the goal
  	 *  delete all nodes on CLOSED since OPEN is now empty
  	 */
	closedList->Free();
  	printf("\n	--->>>No Path Found<<<---");
  	return NULL; // No Path Found
};

//! Defines the heuristic g cost function.
/*! This method calculates the g heuristic cost function.
 * This cost is defined as the cost of travelling from the starting point to the
 * current location.
 * @param n which is the current Node.
 * @return the g cost from the start to this current Node.
 */
double Astar:: gCost(Node *n) 
{
	double cost;
	if(n == NULL || n->parent==NULL)
		return 0.0;
	cost = n->parent->g_value + Dist(n->pose.p,n->parent->pose.p);
	return cost;
};

//! Defines the heuristic h cost function.
/*! This method calculates the h heuristic cost function.
 * This cost is defined as the cost of travelling from the current location
 * to the goal location + penalty on turning too much + penalty on coming clost
 * to obstacles + penalty on chosing to reverse.
 * @param n which is the current Node.
 * @return the g cost from the start to this current Node.
 */
double Astar::hCost(Node *n) 
{
	double h=0,angle_cost=0,obstacle_penalty=0,reverse_penalty=0,delta_d=0;
	if(n == NULL)
		return(0);
	// Using the Euclidean distance
	h = Dist(end.p,n->pose.p);
	//h = 0;
	if (n->parent != NULL) // Adding the Angle cost, we have to uniform the angle representation to the +ve rep or we well get a non sense result
	{
		double a,b;
		a = n->pose.phi;
		b = n->parent->pose.phi;
		angle_cost = fabs(anglediffs(a,b)); // in radians
		delta_d = Dist(n->pose.p,n->parent->pose.p);
	}
	obstacle_penalty = n->nearest_obstacle;
	if(n->direction == BACKWARD)
		reverse_penalty = delta_d;
	
	// 0.555 is the AXLE Length 
//	return ( h*(1 + reverse_penalty ) + 0.555 * angle_cost + obstacle_penalty*delta_d);
	return ( 1 + reverse_penalty )*( h + delta_d + 0.555 * angle_cost + 0.555*angle_cost*obstacle_penalty*delta_d );
};
//! Checks if the search reached the destination.
/*! This method is used to check if the search reached
 * the goal within a predefined proximity #distGoal.
 */
bool Astar :: goalReached (Node *n) 
{
	double angle_diff, delta_d;
	delta_d = Dist(n->pose.p,end.p);
	if (n->direction == FORWARD)
		angle_diff =	anglediff(end.phi,n->pose.phi);
	else
	{
		angle_diff =	anglediff(end.phi,n->pose.phi + M_PI);
	}
	if ( delta_d <= distGoal && angle_diff <= DTOR(60))
	{
//		cout<<" \n Desired Final Orientation ="<<RTOD(end.phi)<<" Current="<<RTOD(n->pose.phi);
//		cout<<"\n Reached Destination with Diff Orientation="<< RTOD(angle_diff);
		return 1;
	}
	return 0;
};
//! Generates the reachable children from the current location.
/*! This method generates the list of reachable children nodes
 * from the current Node. This is done by making sure that by going to 
 * the neigbouring child we will not collid with anything and we will
 * not violate motion constraints
 * TODO: Proper Motion model check and clock/anti-clock wise rotation.
 */
Node *Astar :: makeChildrenNodes(Node *parent)
{
	Point P; 
	Node  *p, *q;
	SearchSpaceNode *temp;
	double start_angle,end_angle,angle,angle_difference,discrete_angle,robot_angle,child_angle,angle_resolution = DTOR(10);
	bool collides = FALSE;
	int direction;
	P.setX(parent->pose.p.x());
	P.setY(parent->pose.p.y());
	Tree t;
	t.location = P;
	if(!searchSpace)
		return NULL;
	temp = searchSpace;
	// Locate the Cell in the Search Space, necessary to determine the neighbours
	while(temp!=NULL)
	{
		//printf("Node has %d children x=%f y=%f",temp->children.size(),temp->pose.x(),temp->pose.y());		
		if (temp->location == P)
			break;
		temp = temp->next;
	}
	if (!temp) 
	{
	    printf("\n	--->>>	Node not found in the search Space ");
		return NULL;
	}
	q = NULL;
	// Check Each neighbour
	//printf("WHAT??? %d x=%f y=%f",temp->children.size(),P.x(),P.y());
	for (unsigned int i=0;i<temp->children.size();i++) 
	{
		/* 
		 * Check what what as the Robot's direction of motion and see
		 * if we it's easier to go forward or backwards to the child
		 */
		if (parent->direction == FORWARD)
			robot_angle = parent->pose.phi;
		else
			robot_angle = parent->pose.phi + M_PI;		
		// What will be our orientation when we go to this child node ?
		angle = ATAN2(temp->children[i]->location,P);
		// How much it differs from our current orientations ?
		angle_difference = anglediff(angle,parent->pose.phi);
		// Are we gonna turn too much ? if yes then why not go backwards ?
		if (angle_difference > DTOR(120))
		{
			//cout<<"\n Angle difference ="<<RTOD(angle_difference)<<" parent angle="<<RTOD(parent->angle)<<" destination angle="<<RTOD(angle);
			direction = parent->direction * -1;
		}
		else
		{
			direction = parent->direction;
		}
		collides = FALSE;
		/* Discreatize the turning space and check for collison
		 * 1- Angle stored in the node is the direction of the PATH (NOT THE ROBOT)
		 * 2- If we were moving Forward then the Robot direction is the same as the Path
		 * 3- If we were moving BackWard then the Robot direction is Path + M_PI
		 * 4- Determine what will the Robot orientation will be at this step
		 * 5- Check for collision detection with a certain resolution
		 */
		if (direction == FORWARD)
			child_angle = angle;
		else
			child_angle = angle + M_PI;
		if(robot_angle  < 0 ) robot_angle  += 2*M_PI;
		if(child_angle  < 0 ) child_angle  += 2*M_PI;
		// Start from the largest angle and go down
		if (robot_angle > child_angle)
		{
			start_angle  = robot_angle;
			end_angle    = child_angle;
		}
		else
		{
			start_angle  = child_angle;
			end_angle    = robot_angle;
		}
    	discrete_angle =  start_angle;
		//cout<<"\n Start is"<<RTOD(start_angle)<<" End angle="<<RTOD(end_angle);
//		angle_difference = anglediff(start_angle,end_angle);
		for (int s=0 ; s <= ceil(angle_difference/angle_resolution); s++)
		{
			if (inObstacle(temp->children[i]->location,discrete_angle))
			{
				collides= true;
				break;
			}
			if(Abs(start_angle - end_angle) >= DTOR(180))
			{
				discrete_angle += angle_resolution;
				if (discrete_angle > 2*M_PI)
					discrete_angle-= 2*M_PI;
				if(discrete_angle > end_angle)
					discrete_angle = end_angle;							
			}
			else
			{
				discrete_angle -= angle_resolution;
				if (discrete_angle < end_angle)
					discrete_angle = end_angle;
			}
		}
		if (!collides) // if after discretization the child still doens't collide then add it
		{
			p = new Node;
			p->pose.p.setX(temp->children[i]->location.x());
			p->pose.p.setY(temp->children[i]->location.y());
			p->direction  =	direction ;
			t.children.push_back(p->pose.p);
			p->nearest_obstacle = temp->children[i]->obstacle_cost;
			p->parent = parent;
			p->pose.phi = angle;
			//p->wheelchair.SetCheckPoints(this->number_of_point_to_check,this->points_to_check);
			p->next = q;
			q = p;
	    	//cout<<"\n NEW CHILD ADDED";
		}
	}
	// Save the search tree so that it can be displayed later
	if (t.children.size() > 0)
		tree.push_back(t);
	return q;
}
// Free node function
void Astar :: freeNode(Node *n) 
{
	delete n;
}

}

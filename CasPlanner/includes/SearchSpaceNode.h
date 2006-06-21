#ifndef SEARCHSPACENODE_H_
#define SEARCHSPACENODE_H_
#include<Point.h>
#include<vector>
using namespace std;
namespace CasPlanner
{

class SearchSpaceNode
	{ 
	public :
		Point location;
		SearchSpaceNode * parent, * next;
		double obstacle_cost;
		vector <SearchSpaceNode *>  children;
		 SearchSpaceNode ();
		~SearchSpaceNode ();
	};
}

#endif /*SEARCHSPACENODE_H_*/

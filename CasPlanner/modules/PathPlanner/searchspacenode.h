#ifndef SEARCHSPACENODE_H_
#define SEARCHSPACENODE_H_
#include<QPointF>
#include<QVector>
using namespace std;

enum {BridgeNode,RegGridNode};

namespace CasPlanner
{

class SearchSpaceNode
	{
	public :
		QPointF location;
		SearchSpaceNode * parent, * next;
		int type;
		double obstacle_cost;
		QVector <SearchSpaceNode *>  children;
		 SearchSpaceNode ();
		~SearchSpaceNode ();
	};
}

#endif /*SEARCHSPACENODE_H_*/

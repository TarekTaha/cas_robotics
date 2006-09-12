#ifndef SEARCHSPACENODE_H_
#define SEARCHSPACENODE_H_
#include<QPointF>
#include<QVector>
using namespace std;
namespace CasPlanner
{

class SearchSpaceNode
	{
	public :
		QPointF location;
		SearchSpaceNode * parent, * next;
		double obstacle_cost;
		QVector <SearchSpaceNode *>  children;
		 SearchSpaceNode ();
		~SearchSpaceNode ();
	};
}

#endif /*SEARCHSPACENODE_H_*/

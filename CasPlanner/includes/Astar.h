#ifndef ASTAR_H_
#define ASTAR_H_

#include <QPointF>
#include <QBitArray>
#include <QVector>
#include <QImage>
//#include <QList>
#include <math.h>
#include "LList.h"
#include "Node.h"
#include "utils.h"
#include "Robot.h"
#include "SearchSpace.h"
#include "interfaceprovider.h"

namespace CasPlanner
{

class Astar: public SearchSpace
{
	private:
		void   FindRoot();
		double gCost(Node *n);
		double hCost(Node *n);
		Node  *MakeChildrenNodes(Node *parent) ;
		bool   goalReached(Node *n);
	public:
		Astar(Robot *,double dG);
		Astar();
		long int MAXNODES;
		double	distGoal;
		Map    * map;
		Pose start,end;
		Robot *robot;
		Node *root, *current, *childList, *curChild, *q, * test,*path, *p;
		LList *openList,*closedList;
		vector <Tree> tree;
		void   FreeNode      (Node *);
		void   ConvertPixel   (QPointF *p);
		void   ConvertToPixel (QPointF *p);
		int    Obstacle   (QPointF p, double angle);
		bool   GoalReached (Node *n);
		Node*  Search(Pose start,Pose end);
		virtual ~Astar();
};

}

#endif /*ASTAR_H_*/

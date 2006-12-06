#ifndef ASTAR_H_
#define ASTAR_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QPointF>
#include <QBitArray>
#include <QVector>
#include <QImage>
//#include <QList>
#include <math.h>
#include "llist.h"
#include "node.h"
#include "utils.h"
#include "robot.h"
#include "searchspace.h"
#include "interfaceprovider.h"

enum{METRIC,PIXEL};
namespace CasPlanner
{

class Astar: public SearchSpace
{
	private:
		void   findRoot();
		double gCost(Node *n);
		double hCost(Node *n);
		Node  *makeChildrenNodes(Node *parent) ;
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
		void   freeNode      (Node *);
		void   convertPix  (QPointF *p);
		void   convert2Pix (QPointF *p);
		int    inObstacle    (QPointF p, double angle);
		bool   goalReached   (Node *n);
		Node*  startSearch   (Pose start,Pose end,int);
		virtual ~Astar();
};

}

#endif /*ASTAR_H_*/

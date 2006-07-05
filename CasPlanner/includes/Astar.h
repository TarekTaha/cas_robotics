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

namespace CasPlanner
{

class Astar: public Robot , public SearchSpace
{
	private:
		void   FindRoot();
		double gCost(Node *n);
		double hCost(Node *n);
		Node  *MakeChildrenNodes(Node *parent) ;
		bool   goalReached(Node *n);
	public:
		Astar(double r_l ,double r_w ,double o_r,double p_s, QString r_m , QPointF r_c);
		Astar();
		int map_height,map_width,MAXNODES;
		double	pixel_size;
		QVector <QBitArray> map;
		Pose start,end;
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

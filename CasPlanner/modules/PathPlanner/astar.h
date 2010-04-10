/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#ifndef ASTAR_H_
#define ASTAR_H_

#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>

#include <QPointF>
#include <QHash>
#include <QBitArray>
#include <QVector>
#include <QImage>
#include <QString>
#include <math.h>
#include "llist.h"
#include "node.h"
#include "utils.h"
#include "robot.h"
#include "searchspace.h"
//#include "interfaceprovider.h"
#include "heuristic.h"
#include "casplannerexception.h"
#include "map.h"

enum{METRIC,PIXEL};
namespace CasPlanner
{

class Astar: public SearchSpace
{
	private:
		void   findRoot() throw(CasPlannerException);
		void   findDest() throw(CasPlannerException);		
		Node  *makeChildrenNodes(Node *parent) ;
	public:
		Astar(Robot *,double dG, QString heuristicType);
		Astar();
		long int MAXNODES;
		QString hType;
		double	distGoal;
		Heuristic *heuristic;
		Map    * map;
		Pose start,end;
		Robot *robot;
		Node *root, *dest, *current, *childList, *curChild, *q, * test,*path, *p;
		LList *openList,*closedList;
		vector <Tree> tree;
		void   setSocialReward(QHash<QString, int>*);
		void   freeNode     (Node *);
		int    inObstacle   (QPointF p, double angle);
		bool   goalReached  (Node *n);
		Node*  startSearch  (Pose start,Pose end,int);
		virtual ~Astar();
};

}

#endif /*ASTAR_H_*/

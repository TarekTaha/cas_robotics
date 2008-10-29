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
#include "heuristic.h"

namespace CasPlanner
{

Heuristic *  Heuristic::factory(QString type) throw(CasPlannerException)
{
	if (type == "Distance")
	{
		cout <<"\n Distance"; fflush(stdout);
		return new DistanceHeuristic;	
	}
	throw (CasPlannerException((char*)"Bad Heuristic Type"));
}

Heuristic *  Heuristic::factory(QString type,QHash<QString, int> *soRe) throw(CasPlannerException)
{
	if (type == "Social")
	{
		cout <<"\n Social";	fflush(stdout);
		return new SocialHeuristic(soRe);
	}
	throw (CasPlannerException((char*)"Bad Heuristic Type"));
}

double SocialHeuristic::gCost(Node *n)
{
	return 0;
}

double SocialHeuristic::hCost(Node *n,Node * end)
{
	double cost=0,h=0,g=0;
	if(n == NULL || n->parent==NULL)
		return 0.0;
	// Using the Euclidean distance
	h = Dist(end->pose.p,n->pose.p);
	g = n->parent->g_value + Dist(n->pose.p,n->parent->pose.p);	
	cost =  h + g;
	cost -= this->socialRewards->value(QString("%1-%2-%3").arg(n->parent->id).arg(n->id).arg(end->id));
	cout<<qPrintable(QString("%1-%2-%3").arg(n->parent->id).arg(n->id).arg(end->id))<<":="<<this->socialRewards->value(QString("%1-%2-%3").arg(n->parent->id).arg(n->id).arg(end->id))<<endl;
	return cost;
}

double DistanceHeuristic::gCost(Node *n)
{
	double cost;
	if(n == NULL || n->parent==NULL)
		return 0.0;
	cost = n->parent->g_value + Dist(n->pose.p,n->parent->pose.p);
	return cost;
}

double DistanceHeuristic::hCost(Node *n,Node * end)
{
	double h=0,angle_cost=0,obstacle_penalty=0,reverse_penalty=0,delta_d=0;
	if(n == NULL)
		return(0);
	// Using the Euclidean distance
	h = Dist(end->pose.p,n->pose.p);
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
	return ( h*(1 + reverse_penalty ) + 0.555 * angle_cost + obstacle_penalty*delta_d);
}

}

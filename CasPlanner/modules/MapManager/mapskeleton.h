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
#ifndef MAPSKELETON_H_
#define MAPSKELETON_H_

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <list>
#include <map>

#include <QPointF> 
#include <QVector>
#include <QString>
#include "utils.h"

/* 
 * This structure Skeleton will contain the map topology structure.
 * For the time being i will use a manual represenation for proof of
 * concept. But the next step will be to be able to automatically generate
 * the Voronoi Structure of the Map
 */

enum {U,D,R,L};

typedef struct _connection
{
	unsigned int nodeIndex; 
	unsigned int direction;
} Connection;

class Vertex
{
	public:	
		QPointF location;
		QVector <Vertex> neighbours;
		QVector <Connection> connections;
		double prob;
		int visits;
		Vertex()
		{
			location.setX(0);
			location.setY(0);
			prob = 0;
			visits = 0;
		};
		Vertex(double x,double y)
		{
			location.setX(x);
			location.setY(y);
		};
		void setLocation(QPointF p)
		{
			this->location = p;
		};
		void setLocation(double x,double y)
		{
			location.setX(x);
			location.setY(y);
		};
		void connect(unsigned int indx,unsigned int direction )
		{
			Connection con; con.direction = direction; con.nodeIndex = indx;
			this->connections.push_back(con);
		};
		QPointF getLocation()
		{
			return this->location;
		};
		bool operator==(const Vertex& v) const
	  	{
	    	return ((this->location.x()== v.location.x()) && (this->location.y()==v.location.y()) && 
	    			(this->prob==v.prob));
	  	};
		
};

class MapSkeleton
{
	public:
		MapSkeleton();
		virtual ~MapSkeleton();
		bool convertGridToLineWithVoronoi(float minThreshold, float maxThreshold, bool filterByCellValue, float valueToSet);
		void generateInnerSkeleton();
		int  getCurrentSpatialState(Pose p);
		double  getDist2SpatialState(Pose P,int stateIndex);
		int numStates;
		int numDestinations;
		void clear();
		void draw();
		QVector <Vertex> verticies;
		QVector <int> destIndexes; 
};

#endif /*MAPSKELETON_H_*/

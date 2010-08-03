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
#include "logger.h"

/*
 * This structure Skeleton will contain the map topology structure.
 * For the time being i will use a manual represenation for proof of
 * concept. But the next step will be to be able to automatically generate
 * the Voronoi Structure of the Map
 */

enum {U,D,R,L};

class Connection
{
public:
    Connection(){}
    Connection(unsigned int _nodeIndex,int _direction):nodeIndex(_nodeIndex),direction(_direction),numberOfVisits(0) {}
    ~Connection(){}
    unsigned int getNumOfVisits() {return numberOfVisits;}
    unsigned int getNodeIndex() {return nodeIndex;}
    int getDirection() {return direction;}
    void incrementNumVisits(int count) {numberOfVisits+=count;}
    void resetNumVisits(){ numberOfVisits = 0;}
private:
    unsigned int nodeIndex;
    // Currently direction is discretized to 90 deg
    // But ideally it can be any Value
    int direction;
    unsigned int numberOfVisits;
};

class Vertex
{
public:
    Vertex()
    {
        location.setX(0);
        location.setY(0);
        prob = 0;
        visits = 0;
    }
    Vertex(double x,double y)
    {
        location.setX(x);
        location.setY(y);
        prob = 0;
        visits = 0;
    }
    void setLocation(QPointF p)
    {
        this->location = p;
    }
    void setLocation(double x,double y)
    {
        location.setX(x);
        location.setY(y);
    }
    void connect(unsigned int indx,unsigned int direction )
    {
        Connection con(indx,direction);
        this->connections.push_back(con);
    }

    int getConnectNumVisits(unsigned int nodeIndex)
    {
        for(int i=0;i<connections.size();i++)
        {
            if(connections[i].getNodeIndex()==nodeIndex)
            {
                return connections[i].getNumOfVisits();
            }
        }
        return -1;
    }

    void incrementConnectionVisits(unsigned int nodeIndex,unsigned int count)
    {
        for(int i=0;i<connections.size();i++)
        {
            if(connections[i].getNodeIndex()==nodeIndex)
            {
                connections[i].incrementNumVisits(count);
                break;
            }
        }
    }

    void resetSegmentVisits()
    {
        for(int i=0;i<connections.size();i++)
        {
            connections[i].resetNumVisits();
        }
    }

    void resetVisits()
    {
        this->visits = 0;
        prob = 0;
    }

    QPointF getLocation()
    {
        return this->location;
    }

    bool operator==(const Vertex& v) const
    {
        return ((this->location.x()== v.location.x()) && (this->location.y()==v.location.y()) &&
                (this->prob==v.prob));
    }

    void setProb(double _prob)
    {
        prob = _prob;
    }

    double getProb()
    {
        return prob;
    }

    void incrementVisits(int count =1)
    {
        visits+=count;
    }

    int getNumVisits()
    {
        return visits;
    }

    QVector <Connection> connections;
private:
    QPointF location;
    double prob;
    int visits;
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
    int getVertexWithLocation(double x, double y);
    int getVertexWithLocation(QPointF p);
    int getSegmentNumVisits(int source, int dest);
    void resetVertexVisits();
    void resetSegmentVisits();
    // Direction is important : source --> dest not equal to dest --> source
    void incrementConnectionVisitsUniDir(unsigned int source,unsigned int dest,unsigned int count=1);
    // Direction is not important : source --> dest is equal to dest --> source
    void incrementConnectionVisitsBiDir(unsigned int vertex1,unsigned int vertex2,unsigned int count=1);
    int getDestIndex(int state);
    int getNumDestinations();
    int getNumVerticies();
    int numStates;
    int numDestinations;
    void clear();
    void draw();
    QVector <Vertex> verticies;
    QVector <int> destIndexes;
};

#endif /*MAPSKELETON_H_*/

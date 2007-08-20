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
 
class Vertex
{
	public:	
		QPointF location;
		QVector <Vertex> neighbours;
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
		void clear();
		void draw();
		QVector <Vertex> verticies;
		QVector <int> destIndexes; 
};

#endif /*MAPSKELETON_H_*/

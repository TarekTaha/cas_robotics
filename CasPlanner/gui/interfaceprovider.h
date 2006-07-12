#ifndef INTERFACEPROVIDER_H
#define INTERFACEPROVIDER_H

#include <QImage>
#include <QVector> 
#include <QPointF>
#include <QBitArray>
#include <math.h>
#include <QMetaType>
#include "utils.h"

//"Design by contract"

class Map 
{
    public: 
        int width; 
        int height;
        double resolution;
        QByteArray rawData; // for OG-Maps
        bool ** data;       // for Planners
        QPointF center;		// Axis Center of the Map
        Map(int width, int height, double resolution,QPointF center)
        {
            this->width   = width; 
            this->height  = height; 
            this->resolution = resolution;
            this->rawData = NULL;
            this->center = center;
			this->data = new bool * [width];
			for(int i=0; i < width; i++)
			{
				data[i] = new bool [height];
				for(int j=0;j < height;j++)
					data[i][j] = false;
			}
        }
        Map(int width, int height, double resolution,  QByteArray rawData)
        {
            this->width   = width; 
            this->height  = height; 
            this->rawData = rawData; 
            this->resolution = resolution;
			this->data = NULL;         
        }        
        Map(): width(0), height(0), resolution(0), rawData(NULL),data(NULL)
        {
            
        }
        ~Map()
        {
        	if(data)
        	{
				for (int i=0; i < width; i++)
				{
					//qDebug("Deleting Row %d",i);
					//fflush(stdout);
		    		delete  [] data[i];
				}
				delete [] data;
				//qDebug("Previous Map Data deleted");
        	}
        }
 
};

class MapProvider
{
    public:
        virtual Map provideMap()=0; 
        virtual ~MapProvider(){};
};

class LaserProvider 
{
    public:
        virtual QVector<QPointF> getLaserScan(int id)=0; 
        virtual ~LaserProvider(){}; 
};

class SpeedProvider 
{
    public:
        virtual void provideSpeed(double &speed, double &turnRate)=0;
        virtual ~SpeedProvider(){};
}; 

class LocationProvider
{
	public:
		virtual void provideLocation(Pose &location)=0;
		virtual ~LocationProvider(){};
};

#endif




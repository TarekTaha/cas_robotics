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
        QBitArray * data;   // for Planners
        Map(int width, int height, double resolution, QBitArray * data)
        {
            this->width   = width; 
            this->height  = height; 
            this->data = data; 
            this->resolution = resolution;
        }
        Map(int width, int height, double resolution,  QByteArray rawData)
        {
            this->width   = width; 
            this->height  = height; 
            this->rawData = rawData; 
            this->resolution = resolution;
        }        
        Map(): width(0), height(0), resolution(0), rawData(NULL),data(NULL)
        {
            
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




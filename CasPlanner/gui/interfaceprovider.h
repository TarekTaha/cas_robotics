#ifndef INTERFACEPROVIDER_H
#define INTERFACEPROVIDER_H

#include <QImage>
#include <QVector> 
#include <QPointF>
#include <math.h>
#include <QMetaType>

// Interfaces for providers. These are for "Design by contract" type things.

#define R2D(x) ((x*180.0/M_PI))
#define D2R(x) ((x*M_PI/180.0))

class Map 
{
    public: 
        int width; 
        int height;
        double resolution;
        QByteArray rawData;
        
        Map(int width, int height, double resolution, QByteArray data)
        {
            this->width   = width; 
            this->height  = height; 
            this->rawData = data; 
            this->resolution = resolution;
        }
        Map(): width(0), height(0), resolution(0), rawData()
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

#endif




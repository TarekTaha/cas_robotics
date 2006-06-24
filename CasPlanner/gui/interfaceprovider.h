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

class SimpleImage 
{
    public: 
        int width; 
        int height; 
        QByteArray rawData;
        
        SimpleImage(int in_width, int in_height, QByteArray in_data)
        {
            width = in_width; 
            height = in_height; 
            rawData = in_data; 
        }
        SimpleImage(): width(0), height(0), rawData()
        {
            
        }
 
};
class ImgProvider 
{
    public:
        virtual SimpleImage provideImg()=0; 
        virtual ~ImgProvider(){};
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




#ifndef INTERFACEPROVIDER_H
#define INTERFACEPROVIDER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QImage>
#include <QVector> 
#include <QPointF>
#include <QBitArray>
#include <math.h>
#include <QMetaType>
#include "utils.h"
#include "map.h"
#include "playerinterface.h"
//"Design by contract"

using namespace PlayerCc;

class MapProvider
{
    public:
        virtual Map provideMap()=0; 
        virtual ~MapProvider(){};
};

class LaserProvider 
{
    public:
        virtual LaserScan getLaserScan()=0; 
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




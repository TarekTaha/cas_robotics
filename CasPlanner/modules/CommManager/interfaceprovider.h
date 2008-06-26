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
#ifndef INTERFACEPROVIDER_H
#define INTERFACEPROVIDER_H

// playerinterface.h include the boost shit at the beginning
#include "playerinterface.h"
#include <QImage>
#include <QVector> 
#include <QPointF>
#include <QBitArray>
#include <math.h>
#include <QMetaType>
#include "utils.h"
#include "map.h"
//"Design by contract"

using namespace PlayerCc;

class MapProvider
{
    public:
        virtual void provideMap(Map&)=0; 
        virtual ~MapProvider(){};
};

class LaserProvider 
{
    public:
        virtual void getLaserScan(LaserScan&)=0; 
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




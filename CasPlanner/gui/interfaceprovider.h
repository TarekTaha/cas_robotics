/***************************************************************************
 *   Copyright (C) 2006 by Waleed Kadous   *
 *   waleed@width   *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

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

enum CameraId { CAMERA_OMNI0, CAMERA_OMNI1, CAMERA_WIDE0, CAMERA_WIDE1, CAMERA_ZOOM0, CAMERA_ZOOM1, CAMERA_THERMAL0, CAMERA_THERMAL1, CAMERA_AUX0, CAMERA_AUX1, CAMERA_AUX2, CAMERA_AUX3, CAMERA_COUNT}; 

Q_DECLARE_METATYPE(CameraId);

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
        virtual SimpleImage provideImg(CameraId camId)=0; 
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




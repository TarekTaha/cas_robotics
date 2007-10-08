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
#ifndef MAP_H_
#define MAP_H_
#include "utils.h"
#include <QByteArray>

class Map 
{
    public:
        int width, height;
        float mapRes;
        Pose global_pose;
        QByteArray rawData; 	// for OG-Maps
        bool    ** grid, **temp;        // for Planners
        QVector <QPointF> pointCloud;
        QPointF center;			// Axis Center of the Map
        Map(int width, int height,float mapRes,QPointF center,Pose p);   
        void scale(int newWidth,int newHeight);      
        Map(Pose p);
        Map(float mapRes,Pose p);
        Map(int width, int height, double resolution,  QByteArray rawData);
        Map();
        ~Map();
		// transfers from pixel coordinate to the main coordinate system
		void convertPix(QPointF  *p); 
		// transfers from main coordinate to the pixel coordinate system
		void convert2Pix(QPointF *p);
};
#endif /*MAP_H_*/

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
#ifndef MAPMANAGER_H_
#define MAPMANAGER_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QObject>
#include <QThread>
#include <QString>
#include <QPointF>
#include <QVector>
#include <QImage>
#include <QRgb>
#include <QBitArray>
#include <QFileDialog>

#include "interfaceprovider.h"
#include "mapskeleton.h"
#include "map.h"

//using namespace defs;
/*! Changes different Map representations into an occupancy
 * grid represented by a bit array. This OG will be sent to 
 * the path planner for planning. This manager currently supports
 * the following input:
 * 1- A QImage which is an RGB image.
 * 2- A Laser scan.
 * 3- Occupancy Grid Map.
 */
class MapManager : public Map, public QObject//public QThread
{
	//Q_OBJECT
	public: 
		MapManager();
		MapManager(QString name,float res,bool negate, Pose p);
		void loadMap(QString name,float res,bool negate,Pose p);
		void generateSkeleton();
		virtual ~MapManager();
		bool mapNegate,skeletonGenerated;
		QString mapName;
		QImage image;
		Map * globalMap;
		MapSkeleton mapSkeleton;		
		Map * provideLaserOG(LaserScan laserScan,double local_dist,double pixel_res,Pose pose);
		Map * providePointCloud(LaserScan laserScan, double loca_dist,Pose robotPose);
		Map * provideMapOG  (QImage image,double pixel_res,bool negate,Pose p);
};

#endif /*MAPMANAGER_H_*/

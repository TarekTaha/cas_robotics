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
#include "interfaceprovider.h"

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
		virtual ~MapManager();
		Map * provideLaserOG(LaserScan laserScan,double local_dist,double pixel_res,Pose pose);
		Map * providePointCloud(LaserScan laserScan, double loca_dist,Pose robotPose);
		Map * provideMapOG  (QImage image,double pixel_res,Pose p,bool negate);
};

#endif /*MAPMANAGER_H_*/

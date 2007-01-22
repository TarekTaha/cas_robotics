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
//#include "mrfmodel.h"
using namespace defs;
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
		SSkelPtr  sskel;		
		//MRFModel * mrfModel;
		Map * provideLaserOG(LaserScan laserScan,double local_dist,double pixel_res,Pose pose);
		Map * providePointCloud(LaserScan laserScan, double loca_dist,Pose robotPose);
		Map * provideMapOG  (QImage image,double pixel_res,bool negate,Pose p);
};

#endif /*MAPMANAGER_H_*/

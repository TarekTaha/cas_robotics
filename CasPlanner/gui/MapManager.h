#ifndef MAPMANAGER_H_
#define MAPMANAGER_H_

#include <QObject>
#include <QString>
#include <QPointF>
#include <QVector>
#include <QImage>
#include <QRgb>
#include <QBitArray>
/* Changes different Map representations into an occupancy
 * grid represented by a bit array. This OG will be sent to 
 * the path planner for planning. This manager currently supports
 * the following input:
 * 1- A QImage which is an RGB image.
 * 2- A Laser scan.
 * 3- Occupancy Grid Map.
 */
class MapManager : public QObject 
{
	//Q_OBJECT
	public:
		MapManager();
		virtual ~MapManager();
		QVector <QBitArray> provideLaserOG(QVector<QPointF> laser_scan);
		QVector <QBitArray> provideMapOG(QImage image);
};

#endif /*MAPMANAGER_H_*/

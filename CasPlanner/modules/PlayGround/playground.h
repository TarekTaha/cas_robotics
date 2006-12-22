#ifndef PLAYGROUND_H_
#define PLAYGROUND_H_

#include <QVector>
#include "navigationtab.h"
#include "mapviewer.h"
#include "robotmanager.h"
#include "configfile.h"

class Navigator;
class NavContainer;
class PlanningManager;
class MapViewer;

class PlayGround: public QObject
{
	Q_OBJECT
	public:
		PlayGround();
		PlayGround(QStringList configFiles);
		int loadImage(QString name);
		virtual ~PlayGround();
		int renderingMethod;
		QString mapName;
		NavContainer *navCon;
		MapViewer    *mapViewer;
		MapManager mapManager;	
		Map * mapData;
		QImage image;
		QVector <RobotManager* > robotPlatforms;
		int  setNavContainer(NavContainer* con);
	public slots:	
		void startRobotsComm();
		void stopRobots();		
    signals:
	    void mapUpdated(Map * mapData);		
};

#endif /*PLAYGROUND_H_*/

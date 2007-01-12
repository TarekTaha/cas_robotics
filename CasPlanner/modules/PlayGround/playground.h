#ifndef PLAYGROUND_H_
#define PLAYGROUND_H_

#include <QVector>
#include "navigationtab.h"
#include "mapviewer.h"
#include "robotmanager.h"
#include "configfile.h"
#include "statusbar.h"

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
		virtual ~PlayGround();
		int renderingMethod;
		NavContainer *navCon;
		MapViewer    *mapViewer;
		MapManager   *mapManager;
		StatusLogger *statusLogger;	
		QVector <RobotManager* > robotPlatforms;
		int  setNavContainer(NavContainer* con);
	public slots:	
		void startRobotsComm();
		void stopRobots();
		void loadMap(QString name,float res,bool negate, Pose p);
		void addMsg(int id,int type,QString msg);				
    signals:
	    void mapUpdated(Map * mapData);		
};

#endif /*PLAYGROUND_H_*/

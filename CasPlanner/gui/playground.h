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
public:
	int renderingMethod;
	QString mapName;
	NavContainer *navCon;
	MapViewer    *mapViewer;	
	QVector <RobotManager* > robotPlatforms;
	int  setNavContainer(NavContainer* con);
	void startRobotsComm();
	void startRobotsComm(int robotId);	
	void stopRobots(int robotId);	
	void stopRobots();
	PlayGround();
	PlayGround(QStringList configFiles);
	virtual ~PlayGround();
};

#endif /*PLAYGROUND_H_*/

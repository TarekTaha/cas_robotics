#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QObject>
#include "navigationtab.h"
#include "CommManager.h"
#include "planningmanager.h"
#include "Navigator.h"
#include "configfile.h"
#include "Robot.h"
#include "mapviewer.h"

class Navigator;
class NavContainer;
class PlanningManager;
class MapViewer;

enum{FORCE_FIELD,VFH,CONFIG_SPACE,NO_AVOID};
class RobotManager : public QObject //: public CommManager, public PlanningManager ,public Navigator
{
	Q_OBJECT
    public:
		RobotManager();
		RobotManager(QStringList configFiles);
		~RobotManager(); 
		int readRobotConfigs(ConfigFile *cf);		
		int readCommManagerConfigs(ConfigFile *cf);
		int readPlannerConfigs(ConfigFile *cf);
		int readNavigatorConfigs(ConfigFile *cf);
		int setNavContainer(NavContainer*);
		int setMapViewer(MapViewer*);
		int start();
		int startPlanner();
		int startNavigator();
		int startComms();
		QString mapName;
		NavContainer *navCon;
		MapViewer *mapViewer;
		CommManager * commManager;
		PlanningManager *planner;
		PathPlanner * local_planner;
		Navigator * navigator;
		Robot * robot;
   public slots:
  		void rePaint(PathPlanner*,Pose *,int *);
};

#endif 


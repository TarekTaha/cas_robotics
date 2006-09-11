#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QObject>
#include "commmanager.h"
#include "planningmanager.h"
#include "navigator.h"
#include "configfile.h"
#include "Robot.h"

class Navigator;
class NavContainer;
class PlanningManager;
class MapViewer;

enum{FORCE_FIELD,VFH,CONFIG_SPACE,NO_AVOID};
enum{OPENGL,PAINTER_2D};

class RobotManager : public QObject//public CommManager, public PlanningManager ,public Navigator
{
	Q_OBJECT
    public:
    	RobotManager();
		RobotManager(ConfigFile *cf,int secId);
		~RobotManager(); 
		int readRobotConfigs(ConfigFile *cf,int secId);
		int readCommManagerConfigs(ConfigFile *cf,int secId);
		int readPlannerConfigs(ConfigFile *cf);
		int readNavigatorConfigs(ConfigFile *cf);
		int start();
		int stop();
		int startPlanner();
		int startNavigator();
		int startComms();
		int renderingMethod;
		CommManager     *commManager;
		PlanningManager *planningManager;
		PathPlanner     *local_planner;
		Navigator       *navigator;
		Robot           *robot;
		bool notPaused,notFollowing;
   public slots:
  		void rePaint(PathPlanner*,Pose *,int *);
};

#endif 


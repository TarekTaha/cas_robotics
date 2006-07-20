#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H

#include <QObject>
#include "navigationtab.h"
#include "CommManager.h"
#include "planningmanager.h"
#include "Navigator.h"
#include "configfile.h"

class Navigator;
class NavContainer;
class PlanningManager;

class RobotManager : public QObject //: public CommManager, public PlanningManager ,public Navigator
{
	Q_OBJECT
    public:
		RobotManager();
		RobotManager(QStringList configFiles);
		~RobotManager(); 
		int readCommManagerConfigs(ConfigFile *cf, int sectionid);
		int readPlannerConfigs(ConfigFile *cf, int sectionid);
		int readNavigatorConfigs(ConfigFile *cf, int sectionid);
		int setNavContainer(NavContainer*);
		int start();
		int startPlanner();
		int startNavigator();
		int startComms();
		QString mapName;
		double pixel_res;
		NavContainer *navCon;
		CommManager * commManager;
		PlanningManager *planner;
		PathPlanner * local_planner;
		Navigator * navigator;
   public slots:
  		void rePaint(PathPlanner*,Pose *,int *);
};

#endif 


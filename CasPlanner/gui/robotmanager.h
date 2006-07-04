#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H

#include <QObject>
#include "CommManager.h"
#include "planningmanager.h"
#include "Navigator.h"
#include "configfile.h"

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
		int start();
		int startPlanner();
		int startNavigator();
		int startComms();
		CommManager * commManager;
		PlanningManager *planner;
		Navigator * navigator;
		
   signals:
        void vFound();
   public slots:
//  		virtual void emergencyStop(); 
    private:
		//bool laserEnabled,ptzEnabled;
		//int  LaserId,ptzId;
};

#endif 


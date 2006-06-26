#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H
#include "CommManager.h"
#include "configfile.h"
class RobotManager: public CommManager //public Planner, public MapManager
{
Q_OBJECT
    public:         
	RobotManager();
	RobotManager(QStringList configFiles);
	~RobotManager(); 
	int readCommManagerConfigs(ConfigFile *cf, int sectionid);
	int readPlannerConfigs(ConfigFile *cf, int sectionid);
	int readMapManagerConfigs(ConfigFile *cf, int sectionid);	
	int start();
	int startPlanner();
	int startMapManager();
	int startComms();
   signals:
        void vFound();
    private:
		//bool laserEnabled,ptzEnabled;
		//int  LaserId,ptzId;
};

#endif 


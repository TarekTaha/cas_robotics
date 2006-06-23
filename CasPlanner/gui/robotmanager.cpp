#include <robotmanager.h>

RobotManager::RobotManager()
{
	// Empty Constructor
}

RobotManager::~RobotManager()
{
	// Empty Destructor
}
/* Read Configuration File and Initialize corresponding Sections :
 *  1- Initialize communication with the Player Server.
 *  2- Create the Specific GUI if required / not supported yet.
 *  3- Initialize Map management layer to provide the planner with maps.
 *  4- Initialize the Path Planning solver with the specified parameters.
 */
RobotManager::RobotManager(QStringList configFiles)
{
    for(int j=0; j < configFiles.size(); j++)
    {
		ConfigFile *cf = new ConfigFile();
		int numSections; 
		cf->Load(configFiles[j].toLocal8Bit());
		numSections = cf->GetSectionCount(); 
		for(int i=0; i < numSections; i++)
		{
		    QString sectionName = cf->GetSectionType(i);
		    if(sectionName == "GUI")
		    {
				//If we want to add more tabs or GUI options
				// For future use
		    }
		    if(sectionName == "Control")
		    {
				//Initialize Control Parameters
		    }
		    if(sectionName == "Robot")
		    {
				readCommManagerConfigs(cf, i); 
		    }
		    if(sectionName == "Planner")
		    {
				//readPlannerConfigs(cf, i); 
		    }
		    if(sectionName == "MapManager")
		    {
				//readMapManagerConfigs(cf, i); 
		    }
		}
    }
}
	        
int RobotManager::readCommManagerConfigs(ConfigFile *cf, int sectionid)
{
    CommManager::config( cf, sectionid);
    return 1;
}

//int RobotManager::readMapManagerConfigs(ConfigFile *cf, int sectionid)
//{
//    MapManager::config( cf, sectionid);
//    return 1;
//}
//
//int RobotManager::readPlannerConfigs(ConfigFile *cf, int sectionid)
//{
//    Planner::config( cf, sectionid);
//    return 1;
//}

int RobotManager::start()
{
	startPlanner();
	startMapManager();
	startComms();
    //connect(cammap, SIGNAL(vFound()), this, SIGNAL(vFound()));
    return 1;
}

int RobotManager::startComms()
{
    qDebug("-> Starting Communication Manager."); 
    CommManager::start();
    qDebug("<- Communication Manager Started.");
	return 1;
}

int RobotManager::startPlanner()
{
//    qDebug("-> Starting Planner."); 
//    Planner::start();
//    qDebug("<- Planner Started.");
    return 1;
}

int RobotManager::startMapManager()
{
//    qDebug("-> Starting Map Manager."); 
//    MapManager::start();
//    qDebug("<- Map Manager Started.");
    return 1;
}


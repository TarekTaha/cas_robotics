#include <robotmanager.h>

RobotManager::RobotManager():
commManager(NULL),
planningManager(NULL),
navigator(NULL),
robot(NULL),
notPaused(true),
notFollowing(true)
{
	// Empty Constructor
}

RobotManager::~RobotManager()
{
	// Empty Destructor
	if(commManager)
		delete commManager;
	if(navigator)
		delete navigator;
	if(robot)
		delete robot;
	if(planningManager)
		delete planningManager;
}
/* Read Configuration File and Initialize corresponding Sections :
 *  1- Initialize communication with the Player Server.
 *  2- Create the Specific GUI if required / not supported yet.
 *  3- Initialize Map management layer to provide the planner with maps.
 *  4- Initialize the Path Planning solver with the specified parameters.
 */
RobotManager::RobotManager(PlayGround *playG,ConfigFile *cf,int secId)
{
	this->playGround = playG;
	connect(playGround,SIGNAL(mapUpdated(Map *)),this,SLOT(updateMap(Map *)));
	readRobotConfigs(cf,secId);
	readCommManagerConfigs(cf,secId); 
	int numSections = cf->GetSectionCount(); 
	for(int i=0; i < numSections; i++)
	{
	    QString sectionName = cf->GetSectionType(i);
	    if(sectionName == "Navigator")
	    {
			readNavigatorConfigs(cf);
	    }
	    if(sectionName == "Planner")
	    {
			readPlannerConfigs(cf); 
	    }
	}
}

void RobotManager::updateMap(Map * mapData)
{
	if(planningManager)
		planningManager->setMap(mapData);
}

int RobotManager::readRobotConfigs(ConfigFile *cf,int secId)
{
	robot = new Robot();
	robot->readConfigs(cf,secId);
    return 1;
}       

int RobotManager::readCommManagerConfigs(ConfigFile *cf,int secId)
{
	commManager = new CommManager(robot);
	commManager->readConfigs(cf,secId);
    return 1;
}

int RobotManager::readNavigatorConfigs(ConfigFile *cf)
{
	navigator = new Navigator(playGround,this);
	navigator->readConfigs(cf);
	return 1;
}

int RobotManager::readPlannerConfigs(ConfigFile *cf)
{
	planningManager = new PlanningManager(this);
	planningManager->readConfigs(cf);
    return 1;
}

int RobotManager::start()
{
	startComms();
	startPlanner();
	startNavigator();
    return 1;
}

int RobotManager::stop()
{
	return 1;
}

int RobotManager::startComms()
{
    commManager->start();
	return 1;
}

int RobotManager::startPlanner()
{
    planningManager->setupPlanner();
    return 1;
}

int RobotManager::startNavigator()
{
    navigator->start();
    return 1;
}



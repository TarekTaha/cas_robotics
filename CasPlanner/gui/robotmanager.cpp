#include <robotmanager.h>

RobotManager::RobotManager():
commManager(NULL),
planningManager(NULL),
local_planner(NULL),
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
RobotManager::RobotManager(ConfigFile *cf,int secId)
{
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
	navigator = new Navigator(this);
	navigator->readConfigs(cf);
	return 1;
}

int RobotManager::readPlannerConfigs(ConfigFile *cf)
{
	planningManager = new PlanningManager(this);
	planningManager->readConfigs(cf);
	planningManager->setupPlanner();
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

void RobotManager::rePaint(PathPlanner *pl,Pose * ps,int *draw)
{
//	if(*draw == LOCALPATH)
//		this->local_planner = pl;
//	navCon->mapPainter->setPathEnabled(1);
//	navCon->mapPainter->drawPath(pl,*ps,draw);
}



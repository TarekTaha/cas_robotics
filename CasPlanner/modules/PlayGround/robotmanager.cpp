#include <robotmanager.h>

RobotManager::RobotManager():
playGround(NULL),		
commManager(NULL),
planningManager(NULL),
navigator(NULL),
intentionRecognizer(NULL),
robot(NULL),
notPaused(true),
notFollowing(true)
{
	// Empty Constructor
}

RobotManager::~RobotManager()
{
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
RobotManager::RobotManager(PlayGround *playG,ConfigFile *cf,int secId):
playGround(playG),		
commManager(NULL),
planningManager(NULL),
navigator(NULL),
intentionRecognizer(NULL),
robot(NULL),
notPaused(true),
notFollowing(true)
{
	connect(playGround,SIGNAL(mapUpdated(Map *)),this,SLOT(updateMap(Map *)));
	connect(this,SIGNAL(addMsg(int,int,QString)),playGround,SLOT(addMsg(int,int,QString)));	
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
	QString logMsg;
	robot = new Robot();
	robot->readConfigs(cf,secId);	
	logMsg.append("\n-> Robot Configurations Read.");
	logMsg.append("\n*********************************************************************");
	logMsg.append(QString("\n\t\t Robot  name:\t%1").arg(robot->robotName));
	logMsg.append(QString("\n\t\t Robot Ip is:\t%1:%2").arg(robot->robotIp).arg(robot->robotPort));
	logMsg.append("\n*********************************************************************");
	emit addMsg(0,INFO,logMsg);
    return 1;
}       

int RobotManager::readCommManagerConfigs(ConfigFile *cf,int secId)
{
	commManager = new CommManager(robot,this->playGround);
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
	startIntentionRecognizer();
    return 1;
}

int RobotManager::stop()
{
	return 1;
}

int RobotManager::startComms()
{
    commManager->start();
//    startIntentionRecognizer();
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

int RobotManager::startIntentionRecognizer()
{
	intentionRecognizer = new IntentionRecognizer(this->playGround,this);
	intentionRecognizer->runRecognition = true;
	intentionRecognizer->start();
	return 1;
}



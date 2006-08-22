#include <robotmanager.h>

RobotManager::RobotManager():
commManager(NULL),
planner(NULL),
local_planner(NULL),
navigator(NULL)
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
		ConfigFile *cf = new ConfigFile("localhost",6665);
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
		    if(sectionName == "Navigator")
		    {
				readNavigatorConfigs(cf);
		    }
		    if(sectionName == "Robot")
		    {
				readCommManagerConfigs(cf); 
		    }
		    if(sectionName == "Planner")
		    {
				readPlannerConfigs(cf); 
		    }
		    if(sectionName == "Map")
		    {
				mapName = cf->ReadString(i, "mapname", "resources//casareaicp.png");
//			   	qDebug("\t\t MAP  name:\t%s", qPrintable(mapName)); 				
		    }		
		}
		delete cf;    
    }
}

int RobotManager::setNavContainer(NavContainer* con)
{
	this->navCon = con;
	return 1;
}	
        
int RobotManager::readCommManagerConfigs(ConfigFile *cf)
{
	commManager = new CommManager;
	commManager->readConfigs(cf);
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
	planner = new PlanningManager;
	planner->readConfigs(cf);
	planner->start();
    return 1;
}

int RobotManager::start()
{
	startComms();
	startPlanner();
	startNavigator();
    return 1;
}

int RobotManager::startComms()
{
    commManager->start();
	return 1;
}

int RobotManager::startPlanner()
{
    planner->start();
    return 1;
}

int RobotManager::startNavigator()
{
    navigator->start();
    return 1;
}

void RobotManager::rePaint(PathPlanner *pl,Pose * ps,int *draw)
{
	if(*draw == LOCALPATH)
		this->local_planner = pl;
	navCon->mapPainter->setPathEnabled(1);
	navCon->mapPainter->drawPath(pl,*ps,draw);
}



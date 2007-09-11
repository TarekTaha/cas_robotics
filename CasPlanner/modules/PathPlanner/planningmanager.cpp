#include "planningmanager.h"

/*! 
 * Constructor, parameters are provided directly rather than
 * read from the configuration file
 */
PlanningManager::PlanningManager(RobotManager *robMan,
								 double pixel_res,
								 double dist_goal,
								 double bridge_len,
								 double bridge_res,
								 double reg_grid,
								 double obst_exp,
								 double conn_rad,
								 double obst_pen,
								 double bridge_conn_rad
									  )
{
	this->pixel_res  = pixel_res;
	this->bridge_len = bridge_len;
	this->bridge_res = bridge_res;
	this->reg_grid = reg_grid;
	this->obst_exp = obst_exp;
	this->reg_grid_conn_rad = conn_rad;
	this->obst_pen = obst_pen;
	this->dist_goal = dist_goal;
	this->bridge_conn_rad = bridge_conn_rad;
	this->pathPlanner = NULL;
	this->robotManager = robMan;
	this->connNodesEnabled  = true;
	this->regGridEnabled    = true;
	this->obstPenEnabled    = true;
	this->expObstEnabled    = true;
	this->bridgeTestEnabled = true;
	this->showTreeEnabled   = false;
	robotManager->robot->setCheckPoints(obst_exp);	
   	connect(this, SIGNAL(addMsg(int,int,QString)), robMan->playGround,SLOT(addMsg(int,int,QString)));	
//	qDebug("Pixel Res in Navigator =%f",this->pixel_res);	
}

PlanningManager::PlanningManager(RobotManager *robMan):
pathPlanner(NULL),
renderTree(false),
robotManager(robMan),
bridgeTestEnabled(true),
connNodesEnabled(true),
regGridEnabled(true),
obstPenEnabled(true),
expObstEnabled(true),
showTreeEnabled(true)
{
   	connect(this, SIGNAL(addMsg(int,int,QString)), robMan->playGround,SLOT(addMsg(int,int,QString)));
}

PlanningManager::~PlanningManager()
{
	
}

void PlanningManager::setRobotManager(RobotManager *rob)
{
	this->robotManager = rob;
}

void PlanningManager:: setBridgeTest(int bt)
{
	//qDebug("BridgeTest is set to %d",bt);
	if(!bt)
		bridgeTestEnabled = false;
	else
		bridgeTestEnabled = true;
}

void PlanningManager:: setConnNodes(int bt)
{
	//qDebug("setConnNodes is set to %d",bt);
	if(!bt)
		connNodesEnabled = false;
	else
		connNodesEnabled = true;	
}
void PlanningManager:: setRegGrid(int bt)
{
	//qDebug("setRegGrid is set to %d",bt);
	if(!bt)
		regGridEnabled = false;
	else
		regGridEnabled = true;
}
void PlanningManager:: setObstPen(int bt)
{
	//qDebug("setObstPen is set to %d",bt);
	if(!bt)
		obstPenEnabled = false;
	else
		obstPenEnabled = true;
}
void PlanningManager:: setExpObst(int bt)
{
	//qDebug("setExpObst is set to %d",bt);
	if(!bt)
		expObstEnabled = false;
	else
		expObstEnabled = true;
}
void PlanningManager:: setShowTree(int bt)
{
	//qDebug("setShowTree is set to %d",bt);
	if(!bt)
		showTreeEnabled = false;
	else
		showTreeEnabled = true;
}
void PlanningManager::setBridgeTestValue(double val)
{
	pathPlanner->setBridgeLen(val);
}
void PlanningManager::setConnNodesValue(double val )
{
	pathPlanner->setConRad(val);
}
void PlanningManager::setRegGridValue(double val)
{
	pathPlanner->setRegGrid(val);
}
void PlanningManager::setObstPenValue(double val)
{
	pathPlanner->setObstDist(val);
}
void PlanningManager::setExpObstValue(double val)
{
	pathPlanner->setExpRad(val);
}
void PlanningManager::setBridgeResValue(double val)
{
	pathPlanner->setBridgeRes(val);
}

void PlanningManager::setMap(Map * mapData)
{
	if(!this->pathPlanner)
		this->setupPlanner();		
	pathPlanner->setMap(mapData);		
}

//void PlanningManager::setMap(LaserScan laserScan,double local_dist,Pose robotLocation)
//{
//	if(!this->pathPlanner)
//		this->setupPlanner();
//	pathPlanner->setMap(provideLaserOG(laserScan,local_dist,pixel_res,robotLocation));
//}

//void PlanningManager::updateMap(LaserScan laserScan,double local_dist,Pose robotLocation)
//{
//	Map *newMap = providePointCloud(laserScan,local_dist,robotLocation);
//	if(!this->pathPlanner)
//		this->setupPlanner();
//	pathPlanner->updateMap(newMap);
//}

void PlanningManager::setStart(Pose start)
{
	this->start = start;
}

void PlanningManager::setEnd(Pose end)
{
	this->end = end;
}

bool PlanningManager::fileExist(const char * fname)
{
	//cout<<fname<<endl;
	struct stat stat_buf;
 	if (stat(fname,&stat_buf) != 0)
 		return false;
  	return (stat_buf.st_mode & S_IFMT) == S_IFREG;
}

void PlanningManager::generateSpace()
{
	QTime timer;
	const char * filename = "SearchSpace.txt";
	if(!this->pathPlanner)
		this->setupPlanner();
	if(pathPlanner->search_space)
	{
		return;
		//pathPlanner->FreeSearchSpace();
	}		
	timer.restart();
	if(fileExist(filename))
	{
		qDebug("Loading Space From file ...");
		pathPlanner->readSpaceFromFile(filename);
		if(expObstEnabled)
			pathPlanner->expandObstacles();
		if(connNodesEnabled)
			pathPlanner->connectNodes();	
		qDebug("File loading took:%f sec",timer.elapsed()/double(1000.00));
	}
	else
	{
		qDebug("Generating Space ...");		
		if(expObstEnabled)
			pathPlanner->expandObstacles();
		if(regGridEnabled)
			pathPlanner->generateRegularGrid();
		if(bridgeTestEnabled)
			pathPlanner->bridgeTest();
		if(obstPenEnabled)
			pathPlanner->addCostToNodes();
		if(connNodesEnabled)
			pathPlanner->connectNodes();	
		pathPlanner->saveSpace2File(filename);
		qDebug("Space Generation took:%f sec",timer.elapsed()/double(1000.00));		
	}
	this->renderTree = true;
	pathPlanner->showConnections();
}
Node * PlanningManager::findPath(int coord)
{
	Node * retval;
	if(!this->pathPlanner)
		this->setupPlanner();	
	if(!pathPlanner->search_space)
	{
		generateSpace();
	}

	retval = pathPlanner->startSearch(start,end,coord);		

	if(retval)
	{
		pathPlanner->printNodeList();
	}
	else
	{
		qDebug("No Path Found");
	}
	return retval;
}

int PlanningManager::readConfigs( ConfigFile *cf)
{
	int numSec; 
	numSec = cf->GetSectionCount(); 
	for(int i=0; i < numSec; i++)
	{
	    QString sectionName = cf->GetSectionType(i);
	    if(sectionName == "Planner")
	    {
		   	bridge_len =			cf->ReadFloat(i, "bridge_len",2);
		   	bridge_res = 			cf->ReadFloat(i, "bridge_res",0.5);
		   	reg_grid =				cf->ReadFloat(i, "reg_grid",0.5);
		   	obst_exp = 				cf->ReadFloat(i, "obst_exp",0.2);
		   	reg_grid_conn_rad =		cf->ReadFloat(i, "reg_grid_conn_rad",0.8);
		   	obst_pen = 				cf->ReadFloat(i, "obst_pen",3);
		   	dist_goal = 			cf->ReadFloat(i, "dist_goal",0.2);   	
		   	bridge_conn_rad =       cf->ReadFloat(i, "bridge_conn_rad",0.5);
	    }
	    if(sectionName == "Map")
	    {
		   	pixel_res =  			cf->ReadFloat(i, "pixel_res",0.05);
		   	negate = 				cf->ReadInt(i, "negate",0);
	    }	    
	}
	robotManager->robot->setCheckPoints(obst_exp);	
	this->setupPlanner();
  	return 1;
}

int PlanningManager::setupPlanner()
{
	QString logMsg;
	if (!pathPlanner)
	{
		logMsg.append("\n-> Starting Planner.");
		logMsg.append("\n\tPlanning Parameters:");
		logMsg.append(QString("\n\t\t\t Pixel Resolution = %1").arg(pixel_res));
		logMsg.append(QString("\n\t\t\t Distance to Goal = %1").arg(dist_goal));
		logMsg.append(QString("\n\t\t\t Bridge Test Lenght = %1").arg(bridge_len));
		logMsg.append(QString("\n\t\t\t Bridge Test Res = %1").arg(bridge_res));
		logMsg.append(QString("\n\t\t\t Bridge Conn Rad = %1").arg(bridge_conn_rad));
		logMsg.append(QString("\n\t\t\t Reg Grid Res  = %1").arg(reg_grid));
		logMsg.append(QString("\n\t\t\t Obstacle Expansion Radius = %1").arg(obst_exp));
		logMsg.append(QString("\n\t\t\t Connection Radius = %1").arg(reg_grid_conn_rad));
		logMsg.append(QString("\n\t\t\t Obstacle Penalty = %1").arg(obst_pen));
																	
//		qDebug("-> Starting Planner."); 
//		qDebug("\tPlanning Parameters:"); 
//    	qDebug("\t\t\t Pixel Resolution = %f",pixel_res); 
//    	qDebug("\t\t\t Distance to Goal = %f",dist_goal); 
//    	qDebug("\t\t\t Bridge Test Lenght = %f",bridge_len); 
//    	qDebug("\t\t\t Bridge Test Res = %f",bridge_res); 
//    	qDebug("\t\t\t Reg Grid Res  = %f",reg_grid); 
//    	qDebug("\t\t\t Obstacle Expansion Radius = %f",obst_exp);         
//    	qDebug("\t\t\t Connection Radius = %f",conn_rad);         
//    	qDebug("\t\t\t Obstacle Penalty = %f",obst_pen);	
		pathPlanner = new PathPlanner(robotManager->robot,
									  dist_goal,
									  bridge_len,
									  bridge_res,
									  reg_grid,
									  obst_exp,
									  reg_grid_conn_rad,
									  obst_pen,
									  bridge_conn_rad);
//		qDebug("->Planner Started.");
		logMsg.append("\n->Planner Started.");	
		emit addMsg(0,INFO,logMsg);	
	}
	return 1;
}

int PlanningManager::stop()
{
  	return 1;
}


#include "planningmanager.h"

/* Constructor, parameters are provided directly rather than
 * read from the configuration file
 */
PlanningManager::PlanningManager(  double robot_length,
									  double robot_width,
									  QString robot_model,
									  QPointF rotation_center,
									  double pixel_res,
									  double bridge_len,
									  double bridge_res,
									  double reg_grid,
									  double obst_exp,
									  double conn_rad,
									  double obst_pen)
{
	this->robot_length = robot_length;
	this->robot_width  = robot_width;
	this->robot_model = robot_model;
	this->rotation_center = rotation_center;
	this->pixel_res  = pixel_res;
	this->bridge_len = bridge_len;
	this->bridge_res = bridge_res;
	this->reg_grid = reg_grid;
	this->obst_exp = obst_exp;
	this->conn_rad = conn_rad;
	this->obst_pen = obst_pen;
}	

PlanningManager::PlanningManager():
bridgeTestEnabled(true),
connNodesEnabled(true),
regGridEnabled(true),
obstPenEnabled(true),
expObstEnabled(true),
showTreeEnabled(true)
{
}

PlanningManager::~PlanningManager()
{
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
void PlanningManager::SetMap(QImage map)
{
	if(!this->pathPlanner)
		this->start();	
	pathPlanner->SetMap(provideMapOG(map));	
}
void PlanningManager::GenerateSpace()
{
	if(!this->pathPlanner)
		this->start();
	if(expObstEnabled)
		pathPlanner->ExpandObstacles();
	if(regGridEnabled)
		pathPlanner->GenerateRegularGrid();
	if(bridgeTestEnabled)
		pathPlanner->BridgeTest();
	if(obstPenEnabled)
		pathPlanner->AddCostToNodes();
	if(connNodesEnabled)
		pathPlanner->ConnectNodes();	
	pathPlanner->ShowConnections();
}
Node * PlanningManager::FindPath(Pose start,Pose end)
{
	Node * retval;
	if(!this->pathPlanner)
		this->start();	
	if(pathPlanner->search_space)
	{
		pathPlanner->FreeSearchSpace();
	}
	GenerateSpace();
	retval = pathPlanner->Search(start,end);
	if(retval)
	{
		pathPlanner->PrintNodeList();
	}
	else
	{
		qDebug("No Path Found");
	}
	return retval;
}

int PlanningManager::config( ConfigFile *cf, int sectionid)
{
   	pixel_res =  			cf->ReadFloat(sectionid, "pixel_res",0.05);
   	bridge_len =			cf->ReadFloat(sectionid, "bridge_len",2);
   	bridge_res = 			cf->ReadFloat(sectionid, "bridge_res",0.5);
   	reg_grid =				cf->ReadFloat(sectionid, "reg_grid",0.5);
   	obst_exp = 				cf->ReadFloat(sectionid, "obst_exp",0.2);
   	conn_rad =				cf->ReadFloat(sectionid, "conn_rad",0.8);
   	obst_pen = 				cf->ReadFloat(sectionid, "obst_pen",3);
   	robot_length = 			cf->ReadFloat(sectionid, "robot_length",1.2);
   	robot_width  = 			cf->ReadFloat(sectionid, "robot_width",0.65);
   	robot_model  = 			cf->ReadString(sectionid, "robot_mode","diff");
	rotation_center.setX(cf->ReadFloat(sectionid, "rotation_x",-0.3));
	rotation_center.setY(cf->ReadFloat(sectionid, "rotation_y",0));
  	return 1;
}

int PlanningManager::start()
{
	qDebug("-> Starting Planner."); 
    qDebug("*********************************************************************");	
   	qDebug("Planning Parameters:"); 
    qDebug("\t\t Pixel Resolution = %f",pixel_res); 
    qDebug("\t\t Bridge Test Lenght = %f",bridge_len); 
    qDebug("\t\t Bridge Test Res = %f",bridge_res); 
    qDebug("\t\t Reg Grid Res  = %f",reg_grid); 
    qDebug("\t\t Obstacle Expansion Radius = %f",obst_exp);         
    qDebug("\t\t Connection Radius = %f",conn_rad);         
    qDebug("\t\t Obstacle Penalty = %f",obst_pen);
    qDebug("\t\t Robot length = %f",robot_length);
    qDebug("\t\t Robot Width  = %f",robot_width);
    qDebug("\t\t Robot Model  = %s",qPrintable(robot_model));
    qDebug("\t\t Robot Center of Rotation x:%f y:%f",rotation_center.x(),rotation_center.y());
    qDebug("*********************************************************************"); 	
	if (!pathPlanner)
		pathPlanner = new PathPlanner(robot_length,
								  robot_width,
								  robot_model,
								  rotation_center,
								  pixel_res,
								  bridge_len,
								  bridge_res,
								  reg_grid,
								  obst_exp,
								  conn_rad,
								  obst_pen);
	qDebug("->Planner Started.");
	return 1;
}

int PlanningManager::stop()
{
  	return 1;
}


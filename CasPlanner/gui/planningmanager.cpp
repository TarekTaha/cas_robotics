#include "planningmanager.h"

PlanningManager::PlanningManager()
{
}

PlanningManager::~PlanningManager()
{
}
Node * PlanningManager::FindPath(QImage map,Pose start,Pose end)
{
	this->start();
	planner->SetMap(provideMapOG(map));
	planner->ExpandObstacles();
	planner->GenerateRegularGrid();
	planner->BridgeTest();
	planner->AddCostToNodes();
	planner->ConnectNodes();
	return planner->Search(start,end);
}
int PlanningManager::config( ConfigFile *cf, int sectionid)
{
    qDebug("\n*********************************************************************");
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
    qDebug("\t\t Robot Model  = %s",robot_model);
    qDebug("\t\t Robot Center of Rotation x:%f y:%f",rotation_center.x(),rotation_center.y());
    qDebug("*********************************************************************"); 
  	return 1;
}
int PlanningManager::start()
{
	if (!planner)
		planner = new PathPlanner(robot_length,
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
}

int PlanningManager::stop()
{
  	return 1;
}


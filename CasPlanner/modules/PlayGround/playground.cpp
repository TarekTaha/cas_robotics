#include "playground.h"

PlayGround::PlayGround()
{
}

PlayGround::PlayGround(QStringList configFiles)
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
				QString render = cf->ReadString(i, "renderingMethod", "OpenGL");
				if(render =="OpenGL")
				{
					this->renderingMethod = OPENGL;
					//qDebug("OpenGL !!!");					
				}
				else if (render =="Painter2D")
				{
					this->renderingMethod = PAINTER_2D;
					//qDebug("2D Painter !!!");										
				}
				else
				{
					qDebug("Unknown Rendering Method !!!");
					exit(1);
				}
			}
		    if(sectionName == "Robot")
		    {
		    	RobotManager *rbm = new RobotManager(this,cf,i);
		    	robotPlatforms.push_back(rbm);
		    }
		    if(sectionName == "Map")
		    {
				mapName = cf->ReadString(i, "mapname", "resources//casareaicpA.png");
		    }		
		}
		delete cf;    
    }
}

int PlayGround::setNavContainer(NavContainer* con)
{
	this->navCon = con;
	return 1;
}	

void PlayGround::startRobotsComm()
{
	for(int i=0;i<robotPlatforms.size();i++)
	    robotPlatforms[i]->startComms();
}

void PlayGround::stopRobots()
{
	for(int i=0;i<robotPlatforms.size();i++)
	    robotPlatforms[i]->stop();
}

PlayGround::~PlayGround()
{
	for(int i=0;i<robotPlatforms.size();i++)
	    delete robotPlatforms[i];
}

//void RobotManager::rePaint(PathPlanner *pl,Pose * ps,int *draw)
//{
//	if(*draw == LOCALPATH)
//		this->local_planner = pl;
//	navCon->mapPainter->setPathEnabled(1);
//	navCon->mapPainter->drawPath(pl,*ps,draw);
//}

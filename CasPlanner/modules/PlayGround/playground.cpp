#include "playground.h"

PlayGround::PlayGround():
navCon(NULL),
mapViewer(NULL),
mapManager(NULL)	
{

}

PlayGround::PlayGround(QStringList configFiles):
navCon(NULL),
mapViewer(NULL),
mapManager(NULL)
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
			
				}
			}
		    if(sectionName == "Map")
		    {
				QString mapName = cf->ReadString(i, "mapname", "resources//casareaicpB.png");
				float mapRes  =   cf->ReadFloat(i, "pixel_res",0.05);
				bool mapNegate  = cf->ReadBool(i, "negate",0);
				loadMap(mapName,mapRes,mapNegate,Pose(0,0,0));
		    }
		    if(sectionName == "Robot")
		    {
		    	RobotManager *rbm = new RobotManager(this,cf,i);
		    	robotPlatforms.push_back(rbm);
		    }
		}	
		delete cf;
    }
}

void PlayGround::addMsg(int id,int type,QString msg)
{
	statusLogger->addStatusMsg(id,type,msg);
}

void PlayGround::loadMap(QString name,float res,bool negate,Pose p)
{
	qDebug("Starting Map Manager"); fflush(stdout);
	if(!mapManager)
		mapManager = new MapManager(name,res,negate,p);
	else
		mapManager->loadMap(name,res,negate,p);
	qDebug("Map Manager Started"); fflush(stdout);		
	emit mapUpdated(mapManager->globalMap);
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

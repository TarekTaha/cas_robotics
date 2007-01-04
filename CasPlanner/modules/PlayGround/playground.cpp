#include "playground.h"

PlayGround::PlayGround()
{
}

PlayGround::PlayGround(QStringList configFiles)
{
	mapData = NULL;
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
				mapName = cf->ReadString(i, "mapname", "resources//casareaicpB.png");
				mapRes  = cf->ReadFloat(i, "pixel_res",0.05);
				if(!loadImage(mapName,mapRes))
				{
					qDebug("Error Loading Image");
					exit(1);
				}
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

int PlayGround::loadImage(QString name,float res)
{
	if(!image.load(name, 0))
	{
		return 0;
	}
	if(mapData)
		delete mapData;
	mapData = mapManager.provideMapOG(image,res,Pose(0,0,0),false);
	qDebug("Map Generated");
	emit mapUpdated(mapData);
	return 1;
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

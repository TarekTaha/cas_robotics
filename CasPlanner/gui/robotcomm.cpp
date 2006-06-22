#include <robotcomm.h>

RobotComm::RobotComm()
{

}

RobotComm::~RobotComm()
{
    // Do nothing
}

RobotComm::RobotComm(QStringList configFiles)
{
    for(int h=0; h < configFiles.size(); h++)
    {
		ConfigFile *cf = new ConfigFile();
		int numSections; 
		cf->Load(configFiles[h].toLocal8Bit());
		numSections = cf->GetSectionCount(); 
		for(int i=0; i < numSections; i++)
		{
		    QString sectionName = cf->GetSectionType(i);
		    // Will be modified if i need dynamic Gui Creation Later on
		    if(sectionName == "gui")
		    {
				//rv->createNewTab(cf, i);  
		    }
		    if(sectionName == "Robot")
		    {
				readRobotConfig(cf, i); 
		    }
		}
    }
}
	        
int RobotComm::readRobotConfig(ConfigFile *cf, int sectionid)
{
    CommManager::config( cf, sectionid);
    laserEnabled = (bool) cf->ReadInt(sectionid, "laserEnabled", 1);
    ptzEnabled = cf->ReadInt(sectionid, "ptzEnabled", 1);
    ptzId = cf->ReadInt(sectionid, "ptzId", 0); 
    if(ptzEnabled)
    {
		player->enablePtz(ptzId);
    }
    return 1;
}

int RobotComm::start()
{
    qDebug("Start Robot connection ..."); 
    CommManager::start();
    qDebug("... connected");
    //connect(cammap, SIGNAL(vFound()), this, SIGNAL(vFound()));
    return 1;
}

void RobotComm::setPtz(double pan, double tilt)
{
    player->setPtz(pan,tilt);
}



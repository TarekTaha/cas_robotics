#include "CommManager.h"

CommManager::CommManager():
Comms(),
player(0)
{
	startConnected = false;
	activateControl = false;
	laserEnabled = false;
	ptzEnabled = false;
	occMapEnabled = false;
	startConnected = false;
	connected = false;
}

CommManager::~CommManager()
{
}

void CommManager::emergencyStop()
{
  player->emergencyStop();
}

void CommManager::emergencyRelease()
{
  	player->emergencyRelease();
}

void CommManager::setSpeed(double i_speed, double i_turnRate)
{
  	player->setSpeed(i_speed, i_turnRate);
}

void CommManager::setLocation(Pose location)
{
	player->setLocation(location);
}

void CommManager::setPtz(double p, double t)
{
	player->setPtz(p,t);
}
void CommManager::setSpeed(double i_speed)
{
  	player->setSpeed(i_speed);
}

void CommManager::setTurnRate(double i_turnRate)
{
  	player->setTurnRate(i_turnRate);
}

double CommManager::getSpeed()
{
  	return player->getSpeed();
}

double CommManager::getTurnRate()
{
  	return player->getTurnRate();
}

Pose CommManager::getLocation()
{
  	return player->getLocation();
}
bool CommManager::getLocalized()
{
	return player->getLocalized();
}

void CommManager::provideSpeed(double &speed, double &turnRate)
{
     speed    = getSpeed();
     turnRate = getTurnRate();
}

void CommManager::provideLocation(Pose &location)
{
	location = getLocation();
}

Map CommManager::provideMap()
{
	return player->provideMap();
}

QVector<QPointF> CommManager::getLaserScan()
{
  	return player->getLaserScan();
}

int CommManager::readConfigs( ConfigFile *cf)
{
	int numSec,cnt;
	numSec = cf->GetSectionCount(); 
	for(int i=0; i < numSec; i++)
	{
	    QString sectionName = cf->GetSectionType(i);
	    if(sectionName == "Robot")
	    {
		   	name =                   cf->ReadString(i, "name", "No-Name");
		   	startConnected =  (bool) cf->ReadInt   (i, "startConnected", 1);
		  	activateControl = (bool) cf->ReadInt   (i, "activateControl", 1);
			playerIp =               cf->ReadString(i, "playerIp", "127.0.0.1");
			playerPort =             cf->ReadInt   (i, "playerPort", 6665);
			cnt = cf->GetTupleCount(i,"lasers");
			if (cnt)
			{
				laserEnabled = true;
				for(int c=0; c<cnt; c++)
				{
					laserIds.push_back(cf->ReadTupleInt(i,"laser_pose",c ,0));							
				}
			}
			else
				laserEnabled = false;
			cnt = cf->GetTupleCount(i,"position");				
			if (cnt)
			{
				activateControl = true;
				for(int c=0; c<cnt; c++)
				{
					positionControlId = cf->ReadTupleInt(i,"position",c ,0);							
				}
			}
			else
				activateControl = false;
			cnt = cf->GetTupleCount(i,"map");				
			if (cnt)
			{
				occMapEnabled = true;
				for(int c=0; c<cnt; c++)
				{
					mapId = cf->ReadTupleInt(i,"map",c ,0);							
				}
			}
			else
				occMapEnabled = false;
			cnt = cf->GetTupleCount(i,"localizer");				
			if (cnt==1)
			{
				localizerEnabled = true;
				for(int c=0; c<cnt; c++)
				{
					localizerId = cf->ReadTupleInt(i,"localizer",c ,0);							
				}
			}
			else
				localizerEnabled = false;				
		  	qDebug("Start Connected is %d",startConnected);
		  	if(startConnected)
			{
				this->start();
			}
	    }
	    if(sectionName == "Planner")
	    {
	    }
	}	
  	return 1;
}

int CommManager::start()
{
    qDebug("-> Starting Communication Manager."); 
    qDebug("*********************************************************************"); 	
    qDebug("Communication Parameters:"); 
   	qDebug("\t\t Robot  name:\t%s", qPrintable(name)); 
    qDebug("\t\t Robot Ip is:\t%s:%d", qPrintable(playerIp),playerPort); 
  	qDebug("\t\t Supported Interfaces:");
  	if (!player)
 	{
	   	player = new PlayerInterface(playerIp, playerPort);
   		connect(player, SIGNAL(newData()), this, SIGNAL(newData()));
 	}
  	//Enable Robot Control ?
  	if(activateControl)
  	{
   	  	qDebug("\t\t\t 	- Position/Drive Contrl."); 
    	player->enableControl(positionControlId);
  	}
  	//Laser
  	if(laserEnabled)
  	{
   	  	qDebug("\t\t\t	- Laser."); 
    	player->setLasers(laserIds);
  	}
  	if(occMapEnabled)
  	{
   	  	qDebug("\t\t\t	- Occupancy Map."); 
    	player->enableMap(mapId);
  	}
  	if(localizerEnabled)
  	{
  		qDebug("\t\t\t	- AMCL Localizer."); 
  		player->enableLocalizer(localizerId);
  	}
  	if(player)
  	{
 		if(player->isRunning())
 		{
 			player->quit();
 			usleep(100000);
 		}
    	player->start();	  	
    	connected = true;
  	}
    qDebug("*********************************************************************"); 
    qDebug("-> Communication Manager Started."); 
    return 1;
}

int CommManager::stop()
{
  	return 1;
}


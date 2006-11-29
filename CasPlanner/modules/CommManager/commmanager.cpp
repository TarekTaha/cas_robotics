#include "commmanager.h"

CommManager::CommManager(Robot *rob):
Comms(),
player(NULL),
robot(rob)
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

void CommManager::gotoGoal(Pose goal)
{
	player->gotoGoal(goal);
}

void CommManager::vfhGoto(Pose goal)
{
	player->vfhGoto(goal);
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

Pose CommManager::getOdomLocation()
{
  	return player->getOdomLocation();
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

LaserScan CommManager::getLaserScan()
{
  	return player->getLaserScan();
}

int CommManager::readConfigs( ConfigFile *cf,int secId)
{
	int cnt;
   	name =                   cf->ReadString(secId, "name", "No-Name");
   	startConnected =  (bool) cf->ReadInt   (secId, "startConnected", 1);
  	activateControl = (bool) cf->ReadInt   (secId, "activateControl", 1);
	playerIp =               cf->ReadString(secId, "playerIp", "127.0.0.1");
	playerPort =             cf->ReadInt   (secId, "playerPort", 6665);
	cnt = cf->GetTupleCount(secId,"lasers");
	if (cnt)
	{
		laserEnabled = true;
		for(int c=0; c<cnt; c++)
		{
			Laser ls;
			int id = cf->ReadTupleInt(secId,"lasers",c ,0);							
			ls.index = id;
			lasers.push_back(ls);							
		}
	}
	else
		laserEnabled = false;
	cnt = cf->GetTupleCount(secId,"position");				
	if (cnt)
	{
		activateControl = true;
		for(int c=0; c<cnt; c++)
		{
			positionControlId = cf->ReadTupleInt(secId,"position",c ,0);							
		}
	}
	else
		activateControl = false;
	cnt = cf->GetTupleCount(secId,"map");				
	if (cnt)
	{
		occMapEnabled = true;
		for(int c=0; c<cnt; c++)
		{
			mapId = cf->ReadTupleInt(secId,"map",c ,0);							
		}
	}
	else
		occMapEnabled = false;
	cnt = cf->GetTupleCount(secId,"localizer");				
	if (cnt==1)
	{
		localizerEnabled = true;
		for(int c=0; c<cnt; c++)
		{
			localizerId = cf->ReadTupleInt(secId,"localizer",c ,0);							
		}
	}
	else
		localizerEnabled = false;		
	cnt = cf->GetTupleCount(secId,"vfh");				
	if (cnt==1)
	{
		vfhEnabled = true;
		for(int c=0; c<cnt; c++)
		{
			vfhId = cf->ReadTupleInt(secId,"vfh",c,0);							
		}
	}
	else
		vfhEnabled = false;							
  	if(startConnected)
	{
		this->start();
	}
  	return 1;
}

int CommManager::start()
{
    qDebug("-> Communication Parameters"); fflush(stdout);
    qDebug("*********************************************************************"); 	
  	if (!player)
 	{
	   	player = new PlayerInterface(robot->robotIp,robot->robotPort);
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
   	  	qDebug("\t\t\t	- Laser:%d.",lasers.size()); fflush(stdout);  	
    	player->setLasers(lasers);
  	}
  	if(occMapEnabled)
  	{
   	  	qDebug("\t\t\t	- Occupancy Map."); fflush(stdout);  	
    	player->enableMap(mapId);
  	}
  	if(localizerEnabled)
  	{
  		qDebug("\t\t\t	- AMCL Localizer."); fflush(stdout);  	
  		player->enableLocalizer(localizerId);
  	}
  	if(vfhEnabled)
  	{
  		qDebug("\t\t\t	- VFH Navigator."); fflush(stdout);  	
  		player->enableVfh(vfhId);
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
    qDebug("-> Communication Manager Started."); fflush(stdout);  	
    return 1;
}

int CommManager::stop()
{
  	return 1;
}


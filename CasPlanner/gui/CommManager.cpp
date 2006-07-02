#include "CommManager.h"

CommManager::CommManager():Comms()
{
//	startConnected = false;
//	activateControl = false;
//	laserEnabled = false;
//	ptzEnabled = false;
//	occMapEnabled = false;
//	startConnected = false;
//	connected = false;
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

void CommManager::provideSpeed(double &speed, double &turnRate)
{
     speed    = getSpeed();
     turnRate = getTurnRate();
}

Map CommManager::provideMap()
{
	return player->provideMap();
}

QVector<QPointF> CommManager::getLaserScan(int laserId)
{
  	return player->getLaserScan(laserId);
}


int CommManager::config( ConfigFile *cf, int sectionid)
{
   	name =                   cf->ReadString(sectionid, "name", "No-Name");
   	startConnected =  (bool) cf->ReadInt   (sectionid, "startConnected", 1);
  	activateControl = (bool) cf->ReadInt   (sectionid, "activateControl", 1);
  	positionControlId =      cf->ReadInt   (sectionid, "positionControlId", 0);
  	laserEnabled =           cf->ReadInt   (sectionid, "laserEnabled", 1);
  	laserId =                cf->ReadInt   (sectionid, "laserId", 0);
	playerIp =               cf->ReadString(sectionid, "playerIp", "127.0.0.1");
	playerPort =             cf->ReadInt   (sectionid, "playerPort", 6665);
	occMapEnabled =   (bool) cf->ReadInt   (sectionid, "occMapEnabled", 1);
  	mapId =                  cf->ReadInt   (sectionid, "mapId", 0);
  	qDebug("Start Connected is %d",startConnected);
  	if(startConnected)
	{
		this->start();
	}
  	return 1;
}
int CommManager::start()
{
    qDebug("-> Starting Communication Manager."); 
    qDebug("\n*********************************************************************"); 	
   	qDebug("Robot  name:\t%s", qPrintable(name)); 
    qDebug("Robot Ip is:\t%s:%d", qPrintable(playerIp),playerPort); 
  	qDebug("Supported Interfaces:");
  	if (!player)
 	{
	   	player = new PlayerInterface(this, playerIp, playerPort);
   		connect(player, SIGNAL(newData()), this, SIGNAL(newData()));
 	}
  	//Enable Robot Control ?
  	if(activateControl)
  	{
   	  	qDebug("	- Position/Drive Contrl."); 
    	player->enableControl(positionControlId);
  	}
  	//Laser
  	if(laserEnabled)
  	{
   	  	qDebug("	- Laser."); 
    	player->enableLaser(0, laserId);
  	}
  	if(occMapEnabled)
  	{
   	  	qDebug("	- Occupancy Map."); 
    	player->enableMap(mapId);
  	}
  	if(player)
  	{
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


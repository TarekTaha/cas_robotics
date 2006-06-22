#include "CommManager.h"

CommManager::CommManager():Comms()
{
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

SimpleImage CommManager::provideImg(CameraId camId)
{
//  if(capThreads[camId] != NULL)
//  {
//    return capThreads[camId]->getImg();
//  }
//  else
//  {
//    qWarning("Warning: Camera %d undefined.", camId);
//    return SimpleImage(0,0,NULL);
//  }
}

QVector<QPointF> CommManager::getLaserScan(int laserId)
{
  return player->getLaserScan(laserId);
}


int CommManager::config( ConfigFile *cf, int sectionid)
{
	qDebug("reading %d", sectionid);
   	name = cf->ReadString(sectionid, "name", "No-Name");
   	qDebug("Setting name to %s", qPrintable(name)); 
   	startConnected = (bool) cf->ReadInt(sectionid, "startConnected", 1);
   	if(startConnected)
   	{
	   playerIp = cf->ReadString(sectionid, "playerIp", "127.0.0.1");
	   playerPort = cf->ReadInt(sectionid, "playerPort", 6665);
	   player = new PlayerInterface(this, playerIp, playerPort);
   	}
  	//Enable Robot Control ?
  	activateControl = (bool) cf->ReadInt(sectionid, "activateControl", 1);
  	positionControlId = cf->ReadInt(sectionid, "positionControlId", 0);
  	if(activateControl)
  	{
    	player->enableDrive(positionControlId);
  	}
  	//Laser
  	laserEnabled = cf->ReadInt(sectionid, "laserEnabled", 1);
  	laserId = cf->ReadInt(sectionid, "laserId", 0);
  	if(laserEnabled)
  	{
    	player->enableLaser(0, laserId);
  	}
  	return 1;
}

void CommManager::provideSpeed(double &speed, double &turnRate)
{
     speed = getSpeed();
     turnRate = getTurnRate();
}
int CommManager::start()
{
    if(startConnected)
    {
        connect(player, SIGNAL(newData()), this, SIGNAL(newData()));
        player->start();
    }
    return 1; 
}

int CommManager::stop()
{
  // Do nothing.
  return 1;
}

void CommManager::setSpeed(double i_speed, double i_turnRate)
{
  player->setSpeed(i_speed, i_turnRate);
}

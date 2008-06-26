/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "commmanager.h"

CommManager::CommManager(Robot *rob,PlayGround * playG):
Comms(),
connected(false),
player(NULL),
robot(rob),
playGround(playG)
{
	startConnected 	= false;
	activateControl = false;
	laserEnabled	= false;
	ptzEnabled		= false;
	occMapEnabled	= false;
	startConnected	= false;
	connected		= false;
	connect(this,SIGNAL(addMsg(int,int,QString)),playGround,SLOT(addMsg(int,int,QString)));
}

CommManager::~CommManager()
{
	this->stop();
	this->disconnect();
}

QVector <DeviceType> * CommManager::getDevices()
{
	if(!player)
	{
//		qDebug("PLAAAAAAAAAAAAAAAAAAAAAYEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEERRRRRRRRRRRR");
//		fflush(stdout);
		return NULL;
	}
	return player->getDevices();
}

void CommManager::stop()
{
	if(player)
  		player->stop();
  	else
  		emit addMsg(0,ERROR,QString("\n1 Communication Interface Not started YET!!!"));
}

void CommManager::stopMotors()
{
	if(player)
		player->setCtrEnabled(false);
}


void CommManager::stopRelease()
{
	if(player)
  		player->stopRelease();
  	else
  		emit addMsg(0,ERROR,QString("\n1a Communication Interface Not started YET!!!"));
}

void CommManager::setSpeechNotification(bool state)
{
	if(player)
  		player->setSpeechNotification(state);
  	else
  		emit addMsg(0,ERROR,QString("\n1b Communication Interface Not started YET!!!"));
}

void CommManager::speechSay(QString voiceM)
{
	if(player)
  		player->speechSay(voiceM);
  	else
  		emit addMsg(0,ERROR,QString("\n1c Communication Interface Not started YET!!!"));
}

void CommManager::disconnect()
{
	if(player->isRunning())
	{
		player->quit();
		usleep(100000);
	}
	connected = false;
}

void CommManager::setSpeed(double i_speed, double i_turnRate)
{
	if(player)	
  		player->setSpeed(i_speed, i_turnRate);
  	else
  		emit addMsg(0,ERROR,QString("\n3 Communication Interface Not started YET!!!"));  			
}
void CommManager::setOdometry(Pose odom)
{
	if(player)
  		player->setOdometry(odom);
  	else
  		emit addMsg(0,ERROR,QString("\n3 Communication Interface Not started YET!!!"));  	 		
}
void CommManager::gotoGoal(Pose goal)
{
	if(player)	
		player->gotoGoal(goal);
  	else
  		emit addMsg(0,ERROR,QString("\n4 Communication Interface Not started YET!!!"));  			
}

void CommManager::vfhGoto(Pose goal)
{	
	if(player)
		player->vfhGoto(goal);
  	else
  		emit addMsg(0,ERROR,QString("\n5 Communication Interface Not started YET!!!"));  	
}

void CommManager::setLocation(Pose location)
{
	if(player)
		player->setLocation(location);
  	else
  		emit addMsg(0,ERROR,QString("\n6 Communication Interface Not started YET!!!"));  	
}

void CommManager::setPtz(double p, double t)
{
	if(player)
		player->setPtz(p,t);
  	else
  		emit addMsg(0,ERROR,QString("\n7 Communication Interface Not started YET!!!"));  		
}
void CommManager::setSpeed(double i_speed)
{
	if(player)
  		player->setSpeed(i_speed);
  	else
  		emit addMsg(0,ERROR,QString("\n8 Communication Interface Not started YET!!!"));  	 		
}

void CommManager::setTurnRate(double i_turnRate)
{
	if(player)
  		player->setTurnRate(i_turnRate);
  	else
  		emit addMsg(0,ERROR,QString("\n9 Communication Interface Not started YET!!!"));  	  		
}

double CommManager::getSpeed()
{
	if(player)
  		return player->getSpeed();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n10 Communication Interface Not started YET!!!"));  		
  		return 0;
  	}  		
}

bool   CommManager::getSpeechNotificaionStatus()
{
	return this->speechEnabled;
}

double CommManager::getTurnRate()
{
	if(player)
  		return player->getTurnRate();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n11 Communication Interface Not started YET!!!"));
  		return 0;
  	}  		
}

int CommManager::getLocalizerType()
{
	if(player)
  		return player->getLocalizerType();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n12 Communication Interface Not started YET!!!"));
  		return 0;
  	}
}

int  CommManager::getJoyStickGlobalDir()
{
	if(player)
  		return player->getJoyStickGlobalDir();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n12 Communication Interface Not started YET!!!"));
  		return 0;
  	}  		
}

int	 CommManager::getJoyStickDir()
{
	if(player)
  		return player->getJoyStickDir();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n13 Communication Interface Not started YET!!!"));
  		return 0;
  	}  		
}
/* 
 * Determines what localizer is available and send the location
 */
Pose CommManager::getLocation()
{
	if(player)
  		return player->getLocation();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n14 Communication Interface Not started YET!!!"));
  		return Pose(0,0,0);
  	}  		
}
/*
 * Sends the latest AMCL localizer's location
 */
Pose CommManager::getAmclLocation()
{
	if(player)
  		return player->getAmclLocation();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n14 Communication Interface Not started YET!!!"));
  		return Pose(0,0,0);
  	}  		
}
/*
 * Sends the Latest ODOM location reported by the underlying Positiong interface
 */
Pose CommManager::getOdomLocation()
{
	if(player)
  		return player->getOdomLocation();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n15 Communication Interface Not started YET!!!"));
  		return Pose(0,0,0);
  	}  		
}
/* 
 * Tells you if the AMCL was able to localize the robot with hight accuracy or not
 */
bool CommManager::getLocalized()
{
	if(player)
		return player->getLocalized();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n16 Communication Interface Not started YET!!!"));
  		return false;
  	}		
}
/*
 * Sends Back the Robot's current Speed and turn Rate
 */
void CommManager::provideSpeed(double &speed, double &turnRate)
{
     speed    = getSpeed();
     turnRate = getTurnRate();
}

void CommManager::provideLocation(Pose &location)
{
	location = getLocation();
}

void CommManager::provideMap(Map &map)
{
	if(player)
		map = player->provideMap();

}

void CommManager::getLaserScan(LaserScan &laserScan)
{
	if(player)
		laserScan = player->getLaserScan();

}

int CommManager::readConfigs( ConfigFile *cf,int secId)
{
	int cnt;
   	name =                   cf->ReadString(secId, "name", "No-Name");
   	startConnected  = (bool) cf->ReadInt   (secId, "startConnected", 1);
  	activateControl = (bool) cf->ReadInt   (secId, "activateControl", 1);
  	speechEnabled   = (bool) cf->ReadInt   (secId, "speechEnabled", 0);
	playerIp =               cf->ReadString(secId, "playerIp", "127.0.0.1");
	playerPort =             cf->ReadInt   (secId, "playerPort", 6665);
	joyStickId =             cf->ReadInt   (secId, "joyStick", -1);
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
	cnt = cf->GetTupleCount(secId,"speech");				
	if (cnt==1)
	{
		speechEnabled = true;
		for(int c=0; c<cnt; c++)
		{
			speechId = cf->ReadTupleInt(secId,"speech",c ,0);							
		}
	}
	else
		speechEnabled = false;		
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
	QString logMsg;
	logMsg.append("-> Starting Communication Manager.\n\tCommunication Parameters:");
  	if (!player)
 	{
	   	player = new PlayerInterface(robot->robotIp,robot->robotPort);
   		connect(player, SIGNAL(newData()), this, SIGNAL(newData()));
   		connect(player, SIGNAL(addMsg(int,int,QString)), playGround,SLOT(addMsg(int,int,QString)));
 	}
  	//Enable Robot Control ?
  	if(activateControl)
  	{
   	  	logMsg.append("\n\t\t 	- Position/Drive Contrl."); 
    	player->enableControl(positionControlId);
  	}
  	//Laser
  	if(laserEnabled)
  	{  
		logMsg.append(QString("\n\t\t	- Laser:%1.").arg(lasers.size()));	  		
    	player->setLasers(lasers);
  	}
  	if(occMapEnabled)
  	{
		logMsg.append(QString("\n\t\t	- Occupancy Map.")); 	
    	player->enableMap(mapId);
  	}
  	if(localizerEnabled)
  	{ 	
		logMsg.append(QString("\n\t\t	- AMCL Localizer."));  		
  		player->enableLocalizer(localizerId);
  	}
  	if(vfhEnabled)
  	{  
		logMsg.append(QString("\n\t\t	- VFH Navigator."));  			
  		player->enableVfh(vfhId);
  	}
  	if(speechEnabled)
  	{  
		logMsg.append(QString("\n\t\t	- VFH Navigator."));  			
  		player->enableSpeech(0);
  	}
	if(joyStickId!=-1)
	{
		player->enableJoyStick(joyStickId);		
	}
  	if(player)
  	{
 		if(player->isRunning())
 		{
 			player->quit();
 			usleep(100000);
 		}
    	player->start(QThread::HighestPriority);	  	
    	connected = true;
  	}
	logMsg.append(QString("\n-> Communication Manager Started."));    
    emit addMsg(0,INFO,logMsg);  	
    connected = true;
    return 1;
}



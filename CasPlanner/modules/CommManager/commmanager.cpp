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

void CommManager::emergencyStop()
{
	if(player)
  		player->emergencyStop();
  	else
  		emit addMsg(0,ERROR,QString("\n1 Communication Interface Not started YET!!!"));
//  		qDebug("Communication Interface Not started YET!!!");
}

void CommManager::emergencyRelease()
{
	if(player)	
  		player->emergencyRelease();
  	else
  		emit addMsg(0,ERROR,QString("\n2 Communication Interface Not started YET!!!"));
//  		qDebug("Communication Interface Not started YET!!!");  		
}

void CommManager::setSpeed(double i_speed, double i_turnRate)
{
	if(player)	
  		player->setSpeed(i_speed, i_turnRate);
  	else
  		emit addMsg(0,ERROR,QString("\n3 Communication Interface Not started YET!!!"));  	
//  		qDebug("Communication Interface Not started YET!!!");  		
}

void CommManager::gotoGoal(Pose goal)
{
	if(player)	
		player->gotoGoal(goal);
  	else
  		emit addMsg(0,ERROR,QString("\n4 Communication Interface Not started YET!!!"));  	
//  		qDebug("Communication Interface Not started YET!!!");		
}

void CommManager::vfhGoto(Pose goal)
{	
	if(player)
		player->vfhGoto(goal);
  	else
  		emit addMsg(0,ERROR,QString("\n5 Communication Interface Not started YET!!!"));  	
//  		qDebug("Communication Interface Not started YET!!!");		
}

void CommManager::setLocation(Pose location)
{
	if(player)
		player->setLocation(location);
  	else
  		emit addMsg(0,ERROR,QString("\n6 Communication Interface Not started YET!!!"));  	
//  		qDebug("Communication Interface Not started YET!!!");		
}

void CommManager::setPtz(double p, double t)
{
	if(player)
		player->setPtz(p,t);
  	else
  		emit addMsg(0,ERROR,QString("\n7 Communication Interface Not started YET!!!"));  	
//  		qDebug("Communication Interface Not started YET!!!");		
}
void CommManager::setSpeed(double i_speed)
{
	if(player)
  		player->setSpeed(i_speed);
  	else
  		emit addMsg(0,ERROR,QString("\n8 Communication Interface Not started YET!!!"));  	
//  		qDebug("Communication Interface Not started YET!!!");  		
}

void CommManager::setTurnRate(double i_turnRate)
{
	if(player)
  		player->setTurnRate(i_turnRate);
  	else
  		emit addMsg(0,ERROR,QString("\n9 Communication Interface Not started YET!!!"));  	
//  		qDebug("Communication Interface Not started YET!!!");  		
}

double CommManager::getSpeed()
{
	if(player)
  		return player->getSpeed();
  	else
  	{
  		emit addMsg(0,ERROR,QString("\n10 Communication Interface Not started YET!!!"));  		
//  		qDebug("Communication Interface Not started YET!!!");
  		return 0;
  	}  		
}

double CommManager::getTurnRate()
{
	if(player)
  		return player->getTurnRate();
  	else
  	{
//  		qDebug("Communication Interface Not started YET!!!");
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
//  		qDebug("Communication Interface Not started YET!!!");
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
//  		qDebug("Communication Interface Not started YET!!!");
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
//  		qDebug("Communication Interface Not started YET!!!");
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
//  		qDebug("Communication Interface Not started YET!!!");
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
//  		qDebug("Communication Interface Not started YET!!!");
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
//  		qDebug("Communication Interface Not started YET!!!");
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

Map CommManager::provideMap()
{
	if(player)
		return player->provideMap();
}

LaserScan CommManager::getLaserScan()
{
	if(player)
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
	QString logMsg;
	logMsg.append("-> Starting Communication Manager.\n\tCommunication Parameters:");
//    qDebug("-> Starting Communication Manager."); fflush(stdout);	
//    qDebug("\tCommunication Parameters:"); fflush(stdout);	
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
//   	  	qDebug("\t\t\t	- Laser:%d.",lasers.size()); fflush(stdout);  
		logMsg.append(QString("\n\t\t	- Laser:%1.").arg(lasers.size()));	  		
    	player->setLasers(lasers);
  	}
  	if(occMapEnabled)
  	{
//   	  	qDebug("\t\t\t	- Occupancy Map."); fflush(stdout); 
		logMsg.append(QString("\n\t\t	- Occupancy Map.")); 	
    	player->enableMap(mapId);
  	}
  	if(localizerEnabled)
  	{
//  		qDebug("\t\t\t	- AMCL Localizer."); fflush(stdout);  	
		logMsg.append(QString("\n\t\t	- AMCL Localizer."));  		
  		player->enableLocalizer(localizerId);
  	}
  	if(vfhEnabled)
  	{
//  		qDebug("\t\t\t	- VFH Navigator."); fflush(stdout);  
		logMsg.append(QString("\n\t\t	- VFH Navigator."));  			
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
//    qDebug("-> Communication Manager Started."); fflush(stdout);
	logMsg.append(QString("\n-> Communication Manager Started."));    
    emit addMsg(0,INFO,logMsg);  	
    connected = true;
    return 1;
}

int CommManager::stop()
{
	if(player->isRunning())
	{
		player->quit();
		usleep(100000);
	}
	connected = false;
  	return 1;
}


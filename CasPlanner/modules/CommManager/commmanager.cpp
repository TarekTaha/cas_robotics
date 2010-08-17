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
#include "robot.h"
#include "statusbar.h"
#include "playground.h"

CommManager::CommManager(Robot *rob,PlayGround * playG):
robot(rob),
playGround(playG)
{
	if (!rob)
		throw CasPlannerException((char*)"CommManager::Null Pointer to Robot");
	if (!playG)
		throw CasPlannerException((char*)"CommManager::Null Pointer to PlayGround");
	startConnected 	= false;
	activateControl = false;
	laserEnabled	= false;
	ptzEnabled		= false;
	occMapEnabled	= false;
	startConnected	= false;
	connect(this,SIGNAL(addMsg(int,int,QString)),playGround,SLOT(addMsg(int,int,QString)));
}

CommManager::~CommManager()
{
	stop();
	disconnect();
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

int CommManager::initialize()
{
	QString logMsg;
	logMsg.append("-> Starting Communication Manager.\n\tCommunication Parameters:");
  	if (!connected)
 	{
	   	connect2Robot(robot->robotIp,robot->robotPort);
 	}
  	else
  	{
  		if(isRunning())
                {
                        Q_EMIT addMsg(0,INFO,"Already Connected and running");
  			return 1;
  		}
  	}
  	//Enable Robot Control ?
  	if(activateControl)
  	{
   	  	logMsg.append("\n\t\t 	- Position/Drive Contrl."); 
    	enableControl(positionControlId);
  	}
  	//Laser
  	if(laserEnabled)
  	{  
		logMsg.append(QString("\n\t\t	- Laser:%1.").arg(lasers.size()));	  		
    	setLasers(lasers);
  	}
  	if(occMapEnabled)
  	{
		logMsg.append(QString("\n\t\t	- Occupancy Map.")); 	
    	enableMap(mapId);
  	}
  	if(localizerEnabled)
  	{ 	
		logMsg.append(QString("\n\t\t	- AMCL Localizer."));  		
  		enableLocalizer(localizerId);
  	}
  	if(vfhEnabled)
  	{  
		logMsg.append(QString("\n\t\t	- VFH Navigator."));  			
  		enableVfh(vfhId);
  	}
  	if(speechEnabled)
  	{
		logMsg.append(QString("\n\t\t	- VFH Navigator."));  			
  		enableSpeech(0);
  	}
	if(joyStickId!=-1)
	{
		enableJoyStick(joyStickId);		
	}
    start(QThread::HighestPriority);
	logMsg.append(QString("\n-> Communication Manager Started."));    
    Q_EMIT addMsg(0,INFO,logMsg);
    return 1;
}



#include "playerinterface.h"

PlayerInterface::PlayerInterface(CommManager *in_comms, QString host, int port):
    comms(in_comms),
    playerHost(host),
    playerPort(port),
    pc(0), 
    ctrEnabled(false),
    positionId(0), 
    drive(0), 
    ptzEnabled(false),
    emergencyStopped(false),
    mapEnabled(false)
{
    laserEnabled[0] = false;
    laserEnabled[1] = false;
}

void PlayerInterface::emergencyStop()
{
    dataLock.lockForWrite(); 
    emergencyStopped = true; 
    dataLock.unlock();
}

void PlayerInterface::emergencyRelease()
{
    dataLock.lockForWrite();
    emergencyStopped = false; 
    dataLock.unlock();
}

void PlayerInterface::setSpeed(double i_speed)
{
    dataLock.lockForWrite();
    speed = i_speed; 
    dataLock.unlock();
}

void PlayerInterface::setTurnRate(double i_turnRate)
{
    dataLock.lockForWrite();
    turnRate = i_turnRate;
    dataLock.unlock();
}

double PlayerInterface::getSpeed()
{
    dataLock.lockForRead();
    double retval = speed; 
    dataLock.unlock(); 
    return retval; 
}

double PlayerInterface::getTurnRate()
{
    dataLock.lockForRead();
    double retval = turnRate; 
    dataLock.unlock(); 
    return retval; 
}

void PlayerInterface::stop()
{
    // Do nothing.  
}

void PlayerInterface::setPtz( double in_pan, double in_tilt)
{
    pan = in_pan;
    tilt = in_tilt; 
}

void PlayerInterface::setSpeed(double i_speed, double i_turnRate)
{
    dataLock.lockForWrite();
    speed = i_speed;
    turnRate = i_turnRate;
    dataLock.unlock(); 
}

void PlayerInterface::enableControl(int posId)
{
    ctrEnabled = true;
    positionId = posId;
}

void PlayerInterface::enablePtz(int in_ptzId)
{
    ptzEnabled=true;
    ptzId=in_ptzId;
}

void PlayerInterface::enableMap(int  in_mapId)
{
    mapEnabled = true;
    mapId = in_mapId;
}

void PlayerInterface::enableLaser(int laser_id, int in_playerLaserId)
{
    laserEnabled[laser_id] = true; 
    playerLaserId[laser_id] = in_playerLaserId; 
}

QVector<QPointF> PlayerInterface::getLaserScan(int Laser_id)
{
    //qDebug("Laser data requested"); 
    if(laserEnabled[Laser_id])
    {
        laser[Laser_id]->Lock(); 
        QVector<QPointF> retval(laser[Laser_id]->scan_count);
        for(int i=0; i< laser[Laser_id]->scan_count; i++)
        {
	    	retval[i] = QPointF(laser[Laser_id]->point[i][0], laser[Laser_id]->point[i][1]);    
		}
    	laser[Laser_id]->Unlock(); 
		//qDebug("Returning ... %d", retval.size());
        return retval;
    }
    else 
    {
        return QVector<QPointF>(0);
    }
}
Map PlayerInterface::provideMap()
{
	Map retval;
    int metadata_offset = (map->height-1)*map->width;
    uint8_t mapid = (uint8_t) map->cells[metadata_offset];
    uint8_t robotid = (uint8_t) map->cells[metadata_offset+1];
    int16_t mapposx, mapposy, mapposphi;
    uint32_t time_secs, time_usecs;
    
    //mapposx = (map->cells[metadata_offset+2] & 0x00ff) << 8;
    //mapposx |= (map->cells[metadata_offset+3] & 0xff);
    
    mapposx = *((int16_t *) (map->cells+metadata_offset+2));
    mapposy = *((int16_t *) (map->cells+metadata_offset+4));
    mapposphi = *((int16_t *) (map->cells+metadata_offset+6));
    time_secs = *((int32_t *) (map->cells+metadata_offset+8));
    time_usecs = *((int32_t *) (map->cells+metadata_offset+12));
//    ogmapdata->timeStamp.seconds = time_secs;
//    ogmapdata->timeStamp.useconds = time_usecs;
//    ogmapdata->origin.p.x = mapposx;
//    ogmapdata->origin.p.y = mapposy;
//    ogmapdata->origin.o = mapposphi;
	retval.width      = map->width;
	retval.height     = map->height-1;
	retval.resolution = map->resolution;
    memcpy(&retval.rawData, map->cells, map->width*(map->height-1));
    return retval;
}

void PlayerInterface::run ()
{
    qDebug("Connecting to %s on %s:%d ... \n",qPrintable(comms->getName()),qPrintable(playerHost), playerPort);
    if(pc)
    {
		delete pc; 
    }
    pc = new PlayerClient(qPrintable(playerHost),playerPort);
    pc->SetFrequency(10);
    qDebug("	--->>> Frequency set to 10 Hz"); 
    int retryCount=0; 
    if(ctrEnabled)
    {
		qDebug("	--->>> Motor Control Interface Engaged"); 
		if(drive)
		{
		    delete drive;  
		}
	        drive = new PositionProxy(pc,positionId,'a');
    }
    qDebug("	--->> Robot now can be Drive by Position Commands"); 
    for(int i=0; i < MAX_LASERS; i++)
    {
        if(laserEnabled[i])
        {
            laser[i] = new LaserProxy(pc,playerLaserId[0], 'r');  
        }
    }
    if(mapEnabled)
    {
    	map = new MapProxy(pc,mapId,'r');
    }
    if(ptzEnabled)
    {
		ptz = new PtzProxy(pc, ptzId, 'a'); 
    }
    qWarning("Testing Player Server for Data Read:");    
    while(pc->Read())
    {
    	qWarning(".");    
		sleep(1);  
		retryCount++; 
    }
    qWarning("	--->>> Test Passed, You can read Data from Player Server Now");    
    qDebug("	--->>> Connection Established"); 
    // Hack around player
    for(int i=0; i < 5; i++)
    {
    	usleep(500000); 
    	while(pc->Read())
    	{
			qWarning("Warning! Could not read from player driver -- sleeping for 1 seconds."); 
			sleep(10);  
			retryCount++; 
    	}
    }
    while(true)
    {
        while(pc->Read())
        {
            qWarning("Can not read from Player Server - Retrying"); 
	    	sleep(1); 
	    	retryCount++; 
        }
    	if(!emergencyStopped)
    	{
	        if(ctrEnabled)
	        {
	            drive->Lock();
	            drive->SetSpeed(speed,turnRate);
	            drive->Unlock();             
	        }
			if(ptzEnabled)
			{
		    	ptz->SetCam(pan,tilt, 1);  
			}
			if(mapEnabled)
			{
				map->GetMap();
			}
    	}
	    else 
	    {
	        qDebug("Stopping Robot NOW ");
	        if(ctrEnabled)
	       {
	            drive->SetSpeed(0,0); 
	            drive->SetMotorState(0);
	        }  
	        qDebug("... Robot Stopped");
	        emergencyStopped = false;
	    }
    	emit newData();
    }
    
}

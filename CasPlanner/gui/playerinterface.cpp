#include "playerinterface.h"

PlayerInterface::PlayerInterface(CommManager *com, QString host, int port):
    playerHost(host),
    playerPort(port),   
    pc(0), 
    comms(com),
    ptzEnabled(false),
    ctrEnabled(false),
    mapEnabled(false),
    localizerEnabled(false),
    localized(false),
    emergencyStopped(false),
    positionId(0), 
    drive(0),
    localizer(0)
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
    double retval = getspeed; 
    dataLock.unlock(); 
    return retval; 
}

double PlayerInterface::getTurnRate()
{
    dataLock.lockForRead();
    double retval = getturnrate; 
    dataLock.unlock(); 
    return retval; 
}

bool PlayerInterface::getLocalized()
{
    dataLock.lockForRead();
    double retval = localized; 
    dataLock.unlock(); 
    return retval; 	
}

Pose PlayerInterface::getLocation()
{
    dataLock.lockForRead();
    Pose retval; 
    if(localizer)
    {
	    retval.p.setX(localizer->hypoths[0].mean[0]);
	    retval.p.setY(localizer->hypoths[0].mean[1]);
	    retval.phi = localizer->hypoths[0].mean[2];
	    if(localizer->hypoths[0].weight>=0.9)
	    	localized = true;
	    else
	    	localized = false;
    }
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
void PlayerInterface::setLocation(Pose location)
{
	pose[0]= location.p.x();
	pose[1]= location.p.y();
	pose[2]= location.phi;
	//cout << "\n Default Pose given to the Localizer X="<<path->location.x()<<" Y="<<path->location.y()<<" Theta="<<path->angle;
	//cout << "\n Tracking Distance="<<tracking_distance<<" Kd="<<kd<<" KTheta="<<kt;
	//Set Covariance Matrix
	pose_var[0][0]=0.5;
	pose_var[0][1]=0.5;
	pose_var[0][2]=0.5;
	pose_var[1][0]=0.5;
	pose_var[1][1]=0.5;
	pose_var[1][2]=0.5;
	pose_var[2][0]=0.5;
	pose_var[2][1]=0.5;
	pose_var[2][2]=DTOR(45);
	if(localizer)
	{
		localizer->SetPose(pose,pose_var);	
		this->location.p.setX(location.p.x());
		this->location.p.setY(location.p.y());
		this->location.phi = location.phi;
	}
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
void PlayerInterface::enableLocalizer(int localizerId)
{
	this->localizerId= localizerId;
	localizerEnabled = true;
}

double PlayerInterface::getClosestObst()
{
	double dist,min_dist = 1000;
	for(int Laser_id = 0; Laser_id < MAX_LASERS; Laser_id++)
	{
		if(laserEnabled[Laser_id])
	    {
	    	//TODO: ADD the appropriate pose translation
			laser[Laser_id]->Lock();
		    for(int i=0; i< laser[Laser_id]->scan_count; i++)
		    {
		    	dist = Dist(QPointF(0,0),QPointF(laser[Laser_id]->point[i][0], laser[Laser_id]->point[i][1]));
		    	if(dist < min_dist)
		    		min_dist = dist;
			}
			laser[Laser_id]->Unlock();
		}
	}
	return min_dist;
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
    //uint8_t mapid = (uint8_t) map->cells[metadata_offset];
    //uint8_t robotid = (uint8_t) map->cells[metadata_offset+1];
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
//	for(int i=0 ;i<map->width*(map->height-1);i++)
//	{
//		if((uint8_t)map->cells[i]>100)
//	    	qDebug("Pixel value is:%u",(uint8_t)map->cells[i]);
//	}
    retval.rawData = QByteArray((const char *) map->cells, map->width*(map->height-1));  
    return retval;
}

void PlayerInterface::run ()
{
    qDebug("/********************************************************************/"); 	
    qDebug("Connecting to Robot Server::");
    qDebug("\t Connecting to %s on %s:%d ...",qPrintable(comms->getName()),qPrintable(playerHost), playerPort);
    if(pc)
    {
		delete pc; 
    }
    pc = new PlayerClient(qPrintable(playerHost),playerPort);
    pc->SetFrequency(10);
    qDebug("\t\t - Frequency Set to 10 Hz"); 
    /* TODO: Proper check for the successfullness of the proxy creation
     */
    if(ctrEnabled)
    {
		if(drive)
		{
		    delete drive;  
		}
        drive = new PositionProxy(pc,positionId,'a');
   		qDebug("\t\t - Motor Control Interface Engaged Successfully"); 
    }
    for(int i=0; i < MAX_LASERS; i++)
    {
        if(laserEnabled[i])
        {
            laser[i] = new LaserProxy(pc,playerLaserId[0], 'r'); 
       		qDebug("\t\t - Laser interface:%d Interface Added Successfully",i);  
        }
    }
    if(mapEnabled)
    {
    	map = new MapProxy(pc,mapId,'r');
		qDebug("\t\t - Map Interface Engaged Successfully");
    }
    if(ptzEnabled)
    {
		ptz = new PtzProxy(pc, ptzId, 'a'); 
		qDebug("\t\t - Pan Tilt unit initialized Successfully");
    }
    if(localizerEnabled)
    {
    	localizer 	= new LocalizeProxy(pc,0,'r');
    	qDebug("\t\t - Localizer Started Successfully");
    }
    qDebug("\t Testing Player Server for Data Read:");    
    while(pc->Read())
    {
    	qWarning(".");    
		sleep(1);
    }
    qDebug("\t\t - Test Passed, You can read Data from Player Server Now");    
    qDebug("\t\t - Connection Established"); 
    qDebug("/********************************************************************/"); 	
	int	mapCounter=0;
    while(true)
    {
        while(pc->Read())
        {
            qWarning("	--->>> Can not read from Player Server - Retrying <<<---"); 
	    	sleep(1); 
        }
    	if(!emergencyStopped)
    	{
	        if(ctrEnabled)
	        {
	            drive->Lock();
	            drive->SetSpeed(speed,turnRate);
	            getspeed = drive->Speed();
	            getturnrate = drive->TurnRate();
	            drive->Unlock();             
	        }
			if(ptzEnabled)
			{
		    	ptz->SetCam(pan,tilt, 1);  
			}
			if(mapEnabled)
			{
				if(((mapCounter++)%10)==0)
					map->GetMap();
			    //qDebug("Map width %d, height %d resolution %f",map->width,map->height,map->resolution);
			}
    	}
	    else 
	    {
	        qDebug("	--->>> Stopping Robot NOW <<<---");
	        if(ctrEnabled)
	       {
	            drive->SetSpeed(0,0); 
	            drive->SetMotorState(0);
	        } 
	        qDebug("	--->>> Robot Stopped <<<---");
	        // temporary fix, needs more thinking once i finalize things
	        emergencyStopped = false;
	    }
    	emit newData();
    }
    
}

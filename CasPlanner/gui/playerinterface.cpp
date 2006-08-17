#include "playerinterface.h"

PlayerInterface::PlayerInterface(QString host, int port):
    playerHost(host),
    playerPort(port),   
    pc(0), 
    ptzEnabled(false),
    ctrEnabled(false),
    mapEnabled(false),
    localizerEnabled(false),
    localized(false),
    emergencyStopped(false),
    velControl(true),
    positionId(0), 
    drive(0),
    localizer(0)
{
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
	velControl = true;    
    speed = i_speed;
    turnRate = i_turnRate;
    dataLock.unlock(); 
}

void PlayerInterface::setSpeed(double i_speed)
{
    dataLock.lockForWrite();
	velControl = true;        
    speed = i_speed; 
    dataLock.unlock();
}

void PlayerInterface::setTurnRate(double i_turnRate)
{
    dataLock.lockForWrite();
	velControl = true;    
    turnRate = i_turnRate;
    dataLock.unlock();
}

void PlayerInterface::gotoGoal(Pose goal)
{
    dataLock.lockForWrite();	
	velControl = false;
	this->goal = goal;	
    dataLock.unlock();	
}

void PlayerInterface::setLocation(Pose loc)
{
	pose[0]= loc.p.x();
	pose[1]= loc.p.y();
	pose[2]= loc.phi;
	//cout << "\n Default Pose given to the Localizer X="<<path->location.x()<<" Y="<<path->location.y()<<" Theta="<<path->angle;
	//cout << "\n Tracking Distance="<<tracking_distance<<" Kd="<<kd<<" KTheta="<<kt;
	//Set Covariance Matrix
	pose_covar[0]=1;
	pose_covar[1]=1;
	pose_covar[2]=DTOR(60);
	if(localizer)
	{
		localizer->SetPose(pose,pose_covar);	
		this->location.p.setX(loc.p.x());
		this->location.p.setY(loc.p.y());
		this->location.phi = loc.phi;
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

void PlayerInterface::setLasers(QVector<int> laserIds)
{
	this->laserIds = laserIds;
}
void PlayerInterface::enableLocalizer(int localizerId)
{
	this->localizerId= localizerId;
	localizerEnabled = true;
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
    Pose retval = location; 
    dataLock.unlock(); 
    return retval; 	
}

Pose PlayerInterface::getOdomLocation()
{
    dataLock.lockForRead();
    Pose retval = odom_location; 
    dataLock.unlock(); 
    return retval; 	
}

QVector<QPointF> PlayerInterface::getLaserScan()
{
	QVector<QPointF> retval;
    dataLock.lockForWrite();
    retval = laserScanPoints;
    dataLock.unlock();
    return retval;
}

Map PlayerInterface::provideMap()
{
	Map retval;
    int metadata_offset = (map->GetHeight()-1)*map->GetWidth();
    //uint8_t mapid = (uint8_t) map->cells[metadata_offset];
    //uint8_t robotid = (uint8_t) map->cells[metadata_offset+1];
    int16_t mapposx, mapposy, mapposphi;
    uint32_t time_secs, time_usecs;
    int8_t * mapdata;
    assert(mapdata = (int8_t*)malloc(sizeof(int8_t) *map->GetHeight()* map->GetWidth()));
    map->GetMap(mapdata);
    //mapposx = (map->cells[metadata_offset+2] & 0x00ff) << 8;
    //mapposx |= (map->cells[metadata_offset+3] & 0xff);
    mapposx =   *((int16_t *) (mapdata+metadata_offset+2));
    mapposy =   *((int16_t *) (mapdata+metadata_offset+4));
    mapposphi = *((int16_t *) (mapdata+metadata_offset+6));
    time_secs = *((int32_t *) (mapdata+metadata_offset+8));
    time_usecs =*((int32_t *) (mapdata+metadata_offset+12));
//    ogmapdata->timeStamp.seconds = time_secs;
//    ogmapdata->timeStamp.useconds = time_usecs;
//    ogmapdata->origin.p.x = mapposx;
//    ogmapdata->origin.p.y = mapposy;
//    ogmapdata->origin.o = mapposphi;
	retval.width      = map->GetWidth();
	retval.height     = map->GetHeight()-1;
	retval.resolution = map->GetResolution();
//	for(int i=0 ;i<map->width*(map->height-1);i++)
//	{
//		if((uint8_t)map->cells[i]>100)
//	    	qDebug("Pixel value is:%u",(uint8_t)map->cells[i]);
//	}
    retval.rawData = QByteArray((const char *) mapdata, map->GetWidth()*(map->GetHeight()-1));
    free(mapdata);
    return retval;
}

void PlayerInterface::run ()
{
    qDebug("/********************************************************************/"); 	
    qDebug("Connecting to Robot Server::");
    qDebug("\t Connecting to %s:%d ...",qPrintable(playerHost), playerPort);
    if(pc)
    {
		delete pc; 
    }
    try
    {
	    pc = new PlayerClient(qPrintable(playerHost),playerPort);
	    /* TODO: Proper check for the successfullness of the proxy creation
	     */
	    if(ctrEnabled)
	    {
			if(drive)
			{
			    delete drive;
			}
	        drive = new Position2dProxy(pc,positionId);
	   		qDebug("\t\t - Motor Control Interface Engaged Successfully"); 
	    }
	    laser.clear();
	    for(int i=0; i < laserIds.size(); i++)
	    {
	    	LaserProxy *lp = new LaserProxy(pc,laserIds[i]);
            laser.push_back(lp);
       		qDebug("\t\t - Laser interface:%d Interface Added Successfully",laserIds[i]);  
	    }
	    if(mapEnabled)
	    {
	    	map = new MapProxy(pc,mapId);
			qDebug("\t\t - Map Interface Engaged Successfully");
	    }
	    if(ptzEnabled)
	    {
			ptz = new PtzProxy(pc, ptzId); 
			qDebug("\t\t - Pan Tilt unit initialized Successfully");
	    }
	    if(localizerEnabled)
	    {
	    	localizer 	= new LocalizeProxy(pc,0);
	    	qDebug("\t\t - Localizer Started Successfully");
	    }
    }
   catch (PlayerCc::PlayerError e)
  	{
    	std::cerr << e << std::endl;
    	return;
  	}
    qDebug("\t Testing Player Server for Data Read:");    
    qDebug("\t\t - Test Passed, You can read Data from Player Server Now");    
    qDebug("\t\t - Connection Established"); 
    qDebug("/********************************************************************/");
//    long int cnt=0;
    while(true)
    {
    	// Read Only if new Data is Available
		//pc->ReadIfWaiting();
		pc->Read();
//		qDebug("\t\t PLAYER THREAD LOOP START %d",++cnt);		
    	if(!emergencyStopped)
    	{
		    for(int laser_indx=0; laser_indx < laserIds.size(); laser_indx++)
		    {
		    	if (laserScanPoints.size())
		    		laserScanPoints.clear();
		        for(uint i=0; i< laser[laser_indx]->GetCount(); i++)
		        {
			    	laserScanPoints.push_back(QPointF(laser[laser_indx]->GetPoint(i).px, laser[laser_indx]->GetPoint(i).py));    
				}
		  	} 
	        if(ctrEnabled)
	        {
	        	player_pose_t ps;
	        	if(velControl)
	        	{
		            drive->SetSpeed(speed,turnRate);
	        	}
	        	else
	        	{
	        		drive->GoTo(goal.p.x(),goal.p.y(),goal.phi);
	        	}
	            getspeed = drive->GetXSpeed();
	            getturnrate = drive->GetYSpeed();
	            ps = drive->GetPose();
	            odom_location.p.setX(drive->GetXPos());
	            odom_location.p.setY(drive->GetYPos());
	            odom_location.phi =  drive->GetYaw();
				//cout<<"\n Current Location X:"<<odom_location.p.x()<<" Y:"<<odom_location.p.y()<<" Theta:"<<odom_location.phi;	            
	        }
			if(ptzEnabled)
			{
		    	ptz->SetCam(pan,tilt, 1);
			}
			if(mapEnabled)
			{
//				map->GetMap();
			    //qDebug("Map width %d, height %d resolution %f",map->width,map->height,map->resolution);
			}
		    if(localizer)
		    {
			    location.p.setX(localizer->GetHypoth(0).mean.px);
			    location.p.setY(localizer->GetHypoth(0).mean.py);
			    location.phi =  localizer->GetHypoth(0).mean.pa;
			    if(localizer->GetHypoth(0).alpha>=0.9)
			    	localized = true;
			    else
			    	localized = false;
		    }			
    	}
	    else
	    {
	        qDebug("	--->>> Stopping Robot NOW <<<---");
			if(ctrEnabled)
	       	{
	        	drive->SetSpeed(0,0);
	        }
	        qDebug("	--->>> Robot Stopped <<<---");
	        // temporary fix, needs more thinking once i finalize things
	        emergencyStopped = false;
	    }
    	emit newData();
    }
}

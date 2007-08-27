#include "playerinterface.h"

PlayerInterface::PlayerInterface(QString host, int port):
    playerHost(host),
    playerPort(port),   
    pc(0), 
    devices(NULL),
    ptzEnabled(false),
    ctrEnabled(false),
    mapEnabled(false),
    localizerEnabled(false),
    localized(false),
    emergencyStopped(false),
    velControl(true),
    positionId(0), 
    drive(NULL),
    wheelChairCommander(NULL),
    localizer(NULL)
{
}

int PlayerInterface::getJoyStickGlobalDir()
{
	dataLock.lockForRead();
	/* the -90 is because the Joystick's coodrinate is rotated 90 in a way that it's
	 * Y-Axis coordinate aligns with the Robot's X-Axis */
	Pose P(joyAxes.x(),joyAxes.y(),NORMALIZE(atan2(joyAxes.y(),joyAxes.x())-DTOR(90)));
	dataLock.unlock();
	P = Trans2Global(P,odom_location);
	double angle = RTOD(P.phi), dirTolerance = 45;//45 degrees means that we will get only N,S,E,W
	if(angle < 0)
		angle += 360;	
	if( joyAxes.x() == 0 && joyAxes.y() == 0)
	{
		return Nothing;
	}
	else if( (360 - dirTolerance) <= angle || angle <= (0 + dirTolerance) ) // 0
	{
		return East;
	}
	else if( (0 + dirTolerance) < angle && angle < (90 - dirTolerance) )
	{
		return NE;
	}
	else if( (90 - dirTolerance) <= angle && angle <= (90 + dirTolerance) ) //90
	{
		return North;
	}
	else if( ( 90 + dirTolerance ) < angle && angle < (180-dirTolerance) )
	{
		return NW;
	}
	else if( (180 - dirTolerance) <= angle && angle <= (180 + dirTolerance) ) // 180
	{
		return West;
	}
	else if( (180 + dirTolerance) < angle && angle < (270 - dirTolerance) )
	{
		return SW;
	}
	else if( (270 - dirTolerance) <= angle && angle <= (270 + dirTolerance) ) //270
	{
		return South;
	}
	else if( (270 + dirTolerance) < angle && angle < ( 360 - dirTolerance) )
	{
		return SE;
	}
	return -1;
}

int PlayerInterface::getJoyStickDir()
{
	/* 
	 * Be AWARE that the Joystick Coordinate Axis is not the same as the Robot's
	 * Coordinate AXIS (The Robot's X-Axis is Forward, while The Joystick Forward
	 * Direction is the Y-Axis) 
	 */
	dataLock.lockForRead();
	double x = joyAxes.x(), y = joyAxes.y();
	dataLock.unlock();
	double angle;
	angle = RTOD(atan2(y,x));
//	printf("\nThe Angle is:%f",angle);
	if(x==0 && y==0)
	{
		return NoInput;
	}
	else if(x>0 && y>0)
	{
		return NE;
	}
	else if( x>0 && y<0)
	{
		return SE;
	}
	else if(x<0 && y>0)
	{
		return NW;
	}
	else if(x<0 && y<0)
	{
		return SW;
	}
	else if(x==0 && y>0)
	{
		return Up;
	}
	else if(x==0 && y<0)
	{
		return Down;
	}
	else if(x>0 && y==0)
	{
		return Right;
	}
	else if(x<0 && y==0)
	{
		return Left;
	}
	return -1;
}

void PlayerInterface::checkForWheelChair()
{
	QVector<DeviceType> * dev = getDevices();
	for(int i=0;i< dev->size(); i++)
	{
		if((*dev)[i].driverName == "WheelchairDriver")
		{
			if (wheelChairCommander)
				delete wheelChairCommander;
			wheelChairCommander = new WheelChairProxy(pc,0);
		  	printf("\n Turning ON WheelChair"); fflush(stdout);
		  	wheelChairCommander->setPower(ON);
		  	wheelChairCommander->setMode(AUTO);
		  	wheelChairCommander->soundHorn(1000); // for a second
		  	printf("\n WheelChair is on"); fflush(stdout);			
		  	logMsg.append(QString("\n\t\t - Wheelchair Driver Engaged"));
		}
	}
}

QVector<DeviceType> * PlayerInterface::getDevices()
{
  	// Connect to the server
  	dataLock.lockForWrite();
  	if(devices)
  		devices->clear();
  	else
  		devices = new QVector<DeviceType>;
  	//printf("Connecting to [%s:%d]\n", qPrintable(host), port);
  	client = playerc_client_create(NULL, qPrintable(this->playerHost), this->playerPort);
  	if (playerc_client_connect(client) != 0)
  	{
    	printf("%s", playerc_error_str());
    	return NULL;
  	}	
  	// Get the available devices.
  	if (playerc_client_get_devlist(client) != 0)
  	{
    	printf("%s", playerc_error_str());
    	return devices;
  	}
  	DeviceType device;
  	for (int i = 0; i < client->devinfo_count; i++)
  	{
    	device.setAddress(client->devinfos[i].addr);
    	device.setName(strdup(client->devinfos[i].drivername));
		switch(client->devinfos[i].addr.interf)
		{
			case PLAYER_LASER_CODE :
				for(int j=0;j<lasers.size();j++)
				{
					if(lasers[j].lp && (client->devinfos[i].addr.index == j))
						device.subscribed = true;
				}
				break;
			case PLAYER_MAP_CODE :
				if(this->map)
					device.subscribed = true;
				break;				
			case PLAYER_POSITION2D_CODE:
				if(this->drive && (client->devinfos[i].addr.index == 0))
					device.subscribed = true;
				else if((this->vfh && (client->devinfos[i].addr.index == 1)))
					device.subscribed = true;
				break;		
			case PLAYER_LOCALIZE_CODE:
				if(this->localizer)
					device.subscribed = true;	
				break;		
			case PLAYER_PTZ_CODE:
				if(this->ptz)
					device.subscribed = true;	
				break;						
			default:
				device.subscribed = false;
		}
		devices->push_back(device);
  	}  	
  	dataLock.unlock();
  	return devices;
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

void PlayerInterface::vfhGoto(Pose goal)
{
    dataLock.lockForWrite();
	velControl = false;
	this->vfhGoal = goal;
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

void PlayerInterface::enableVfh(int in_vfhId)
{
	vfhEnabled = true;
	vfhId = in_vfhId;
}

void PlayerInterface::enableMap(int  in_mapId)
{
    mapEnabled = true;
    mapId = in_mapId;
}

void PlayerInterface::setLasers(QVector<Laser> lasers)
{
	this->lasers = lasers;
//	qDebug("\n I got Lasers%d and now i have %d Lasers",lasers.size(),this->lasers.size());
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

LaserScan PlayerInterface::getLaserScan()
{
	LaserScan retval;
    dataLock.lockForWrite();
    retval = laserScan;
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
	retval.mapRes 	  = map->GetResolution();
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
	logMsg.append("\n/********************************************************************/");
	logMsg.append("\nConnecting to Robot Server::");
	logMsg.append(QString("\n\t Connecting to %1:%2 ...").arg(qPrintable(playerHost)).arg(playerPort));
//    qDebug("/********************************************************************/"); 	
//    qDebug("Connecting to Robot Server::");
//    qDebug("\t Connecting to %s:%d ...",qPrintable(playerHost), playerPort);
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
			checkForWheelChair();
	        drive = new Position2dProxy(pc,positionId);
//	   		qDebug("\t\t - Motor Control Interface Engaged Successfully, ID:%d",positionId);
			logMsg.append(QString("\n\t\t - Motor Control Interface Engaged Successfully, ID:%1").arg(positionId));	   		 
	    }
	    for(int i=0; i < lasers.size(); i++)
	    {
	    	player_pose_t 	lp_pose;
	    	lp_pose.px = 0;
	    	lp_pose.py = 0;
	    	lp_pose.pa = 0;
	    	lasers[i].lp = new LaserProxy(pc,lasers[i].index);
	    	//lp_pose = lasers[i].lp->GetPose();
	    	lasers[i].pose.p.setX(lp_pose.px);
	    	lasers[i].pose.p.setY(lp_pose.py);	    	
	    	lasers[i].pose.phi = lp_pose.pa;
	    	//qDebug("Laser Pose X:%f Y:%f Phi:%f",lasers[i].pose.p.x(),lasers[i].pose.p.y(),lasers[i].pose.phi);	    	
//       		qDebug("\t\t - Laser interface:%d Interface Added Successfully",lasers[i].index);
			logMsg.append(QString("\n\t\t - Laser interface:%1 Interface Added Successfully").arg(lasers[i].index));  
	    }
	    if(mapEnabled)
	    {
	    	map = new MapProxy(pc,mapId);
//			qDebug("\t\t - Map Interface Engaged Successfully, ID:%d",cc);
			logMsg.append(QString("\n\t\t - Map Interface Engaged Successfully, ID:%1").arg(mapId));			
	    }
	    if(ptzEnabled)
	    {
			ptz = new PtzProxy(pc, ptzId); 
//			qDebug("\t\t - Pan Tilt unit initialized Successfully ID:%d",ptzId);
			logMsg.append(QString("\n\t\t - Pan Tilt unit initialized Successfully ID:%1").arg(ptzId));			
	    }
	    if(localizerEnabled)
	    {
	    	localizer 	= new LocalizeProxy(pc,0);
//	    	qDebug("\t\t - Localizer Started Successfully ID:%d",0);
			logMsg.append(QString("\n\t\t - Localizer Started Successfully ID:%1").arg(0));	    	
	    }
	    if(vfhEnabled)
	    {
	    	vfh 	= new Position2dProxy(pc,vfhId);
//	    	qDebug("\t\t - Vfh Started Successfully ID:%d",vfhId);
			logMsg.append(QString("\n\t\t - Vfh Started Successfully ID:%1").arg(vfhId));	    	
	    }	   
	    /* This is temp until the wheelchair interface is added.*/
	    joyStickId = 3;
    	joyStick = new Position2dProxy(pc,joyStickId);
    }
   catch (PlayerCc::PlayerError e)
  	{
    	std::cerr << e << std::endl;
    	return;
  	}
	logMsg.append(QString("\n\t Testing Player Server for Data Read:"));  	
	logMsg.append(QString("\n\t\t - Test Passed, You can read Data from Player Server Now"));
	logMsg.append(QString("\n\t\t - Connection Established"));
	logMsg.append(QString("\n/********************************************************************/"));	
//    qDebug("\t Testing Player Server for Data Read:");    
//    qDebug("\t\t - Test Passed, You can read Data from Player Server Now");    
//    qDebug("\t\t - Connection Established"); 
//    qDebug("/********************************************************************/");
    emit addMsg(0,INFO,logMsg);
//    long int cnt=0;
	Timer timer;
    while(true)
    {
    	//qDebug("Loop Time is:%f QTimer:%d",timer.elapsed(),tt.elapsed()); fflush(stdout);
    	timer.restart();
    	// Read Only if new Data is Available
		//pc->ReadIfWaiting();
		pc->Read();		
    	if(!emergencyStopped)
    	{
		    for(int laser_indx=0; laser_indx < lasers.size(); laser_indx++)
		    {
		    	if (laserScan.points.size())
		    		laserScan.points.clear();
		    	laserScan.laserPose.p.setX(lasers[laser_indx].pose.p.x());
		    	laserScan.laserPose.p.setY(lasers[laser_indx].pose.p.y());		    	
		    	laserScan.laserPose.phi =  lasers[laser_indx].pose.phi;		    	
		        for(uint i=0; i< lasers[laser_indx].lp->GetCount(); i++)
		        {
			    	laserScan.points.push_back(QPointF(lasers[laser_indx].lp->GetPoint(i).px, lasers[laser_indx].lp->GetPoint(i).py));    
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
//	        		drive->GoTo(goal.p.x(),goal.p.y(),goal.phi);
//					qDebug("VFH recieved a goto X:%f Y:%f Phi%f",vfhGoal.p.x(),vfhGoal.p.y(),vfhGoal.phi);
	        		vfh->GoTo(vfhGoal.p.x(),vfhGoal.p.y(),vfhGoal.phi);	        		
	        	}
	            getspeed = drive->GetXSpeed();
//	            getturnrate = drive->GetYSpeed();
	            getturnrate = drive->GetYawSpeed();	            
	            ps = drive->GetPose();
	            odom_location.p.setX(drive->GetXPos());
	            odom_location.p.setY(drive->GetYPos());
	            odom_location.phi =  drive->GetYaw();
	            joyAxes.setX(joyStick->GetXPos());
	            joyAxes.setY(joyStick->GetYPos());
//	            int dir = getJoyStickDir();
//	            int globalDir = getJoyStickGlobalDir();
//	            printf("\nDirection=%d Global Dir=%d",dir,globalDir);
//	            cout<<"\n Current Location X:"<<joyAxes.x()<<" Y:"<<joyAxes.y();
//				cout<<"\n Current Location X:"<<odom_location.p.x()<<" Y:"<<odom_location.p.y()<<" Theta:"<<RTOD(odom_location.phi);	            
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

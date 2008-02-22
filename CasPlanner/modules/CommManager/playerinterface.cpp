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
    vfhEnabled(false),
    positionId(0), 
	drive(NULL),
	vfh(NULL),
	joyStick(NULL),
    wheelChairCommander(NULL),
    map(NULL),
	ptz(NULL),
	localizer(NULL),
	speechP(NULL),
	speed(0),
	turnRate(0)
{
	connect(this,SIGNAL(terminated()),this,SLOT(terminateMissions()));
}

void PlayerInterface::terminateMissions()
{
	qDebug("\n\nTERMINTED!!!!\n\n"); fflush(stdout);
}

PlayerInterface::~PlayerInterface()
{
	setSpeed(0,0);
	clearResources();
}

int PlayerInterface::getJoyStickGlobalDir()
{
	dataLock.lockForRead();
	/* the -90 is because the Joystick's coodrinate is rotated 90 in a way that it's
	 * Y-Axis coordinate aligns with the Robot's X-Axis */
	Pose P(joyAxes.x(),joyAxes.y(),NORMALIZE(atan2(joyAxes.y(),joyAxes.x())-DTOR(90)));
	dataLock.unlock();
	if (localizerType == ODOM)
		P = Trans2Global(P,odomLocation);
	else
		P = Trans2Global(P,amclLocation);	
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
//		  	wheelChairCommander->soundHorn(1000);
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
  	if(!pc)
  	{
  		perror("\n Please Connect to the Robot First");
  		return NULL;
  	}
  	typedef std::list <playerc_device_info_t>  devList; 
  	DeviceType device;
  	pc->RequestDeviceList();
  	devList m_devList= pc->GetDeviceList();
  	for(devList::iterator iter = m_devList.begin(); iter != m_devList.end(); iter++)
  	{
  		device.setAddress(iter->addr);
    	device.setName(strdup(iter->drivername));
		switch(iter->addr.interf)
		{
			case PLAYER_LASER_CODE :
				for(int j=0;j<lasers.size();j++)
				{
					if(lasers[j].lp && (iter->addr.index == j))
						device.subscribed = true;
					else
						device.subscribed = false;
				}
				break;
			case PLAYER_MAP_CODE :
				if(this->map)
					device.subscribed = true;
				break;				
			case PLAYER_POSITION2D_CODE:
				if(this->drive && (iter->addr.index == 0))
					device.subscribed = true;
				else if((this->vfh && (iter->addr.index == 1)))
					device.subscribed = true;
				else
						device.subscribed = false;					
				break;		
			case PLAYER_LOCALIZE_CODE:
				if(this->localizer)
					device.subscribed = true;
				else
					device.subscribed = false;					
				break;		
			case PLAYER_PTZ_CODE:
				if(this->ptz)
					device.subscribed = true;
				else
					device.subscribed = false;						
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

int PlayerInterface::getLocalizerType()
{
    dataLock.lockForRead();
    int retval = localizerType; 
    dataLock.unlock(); 
    return retval; 	
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

void PlayerInterface::setOdometry(Pose odom)
{
	if(this->drive)
	{
		drive->SetOdometry(odom.p.x(),odom.p.y(),odom.phi);
	}
}
 
void PlayerInterface::gotoGoal(Pose goal)
{
    dataLock.lockForWrite();
	velControl = false;
	this->goal = goal;
    dataLock.unlock();
}

Pose PlayerInterface::localizeToPosition(Pose localizerWayPoint)
{
	Pose odomWayPoint;
	double offset_x, offset_y, offset_a;
  	double lx_rot, ly_rot;

  	offset_a = anglediffs(this->odomLocation.phi,this->amclLocation.phi);
  	lx_rot = this->amclLocation.p.x() * cos(offset_a) - this->amclLocation.p.y() * sin(offset_a);
  	ly_rot = this->amclLocation.p.x() * sin(offset_a) + this->amclLocation.p.y() * cos(offset_a);

  	offset_x = this->odomLocation.p.x() - lx_rot;
  	offset_y = this->odomLocation.p.y() - ly_rot;

  	odomWayPoint.p.setX(localizerWayPoint.p.x() * cos(offset_a) - localizerWayPoint.p.y() * sin(offset_a) + offset_x);
  	odomWayPoint.p.setY(localizerWayPoint.p.x() * sin(offset_a) + localizerWayPoint.p.y() * cos(offset_a) + offset_y);
  	odomWayPoint.phi = localizerWayPoint.phi + offset_a;
  	return odomWayPoint;
}

void PlayerInterface::vfhGoto(Pose goal)
{
    dataLock.lockForWrite();
	velControl = false;
	/*
	 * In case we are localizing Globally then the waypoint should be
	 * translated from the global frame to the frame of the underlying
	 * Position driver.
	 */
	if(this->localizerType==AMCL)
	{
		this->vfhGoal = localizeToPosition(goal);		
	}
	else
	{
		this->vfhGoal = goal;
	}
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
	pose_covar[0]=0.5*0.5;
	pose_covar[1]=0.5*0.5;
	pose_covar[2]=(M_PI/6.0)*(M_PI/6.0);
	if(localizer)
	{
		localizer->SetPose(pose,pose_covar);	
		this->amclLocation.p.setX(loc.p.x());
		this->amclLocation.p.setY(loc.p.y());
		this->amclLocation.phi = loc.phi;
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
    Pose retval;
    if(localizerType == AMCL)
    {
    	retval = amclLocation;
//    	printf("\n AMCL localizer");
    }
   	else
   	{
   		retval = odomLocation;
// 		printf("\n ODOM localizer");
   	}
    dataLock.unlock(); 
    return retval; 		
}

Pose PlayerInterface::getAmclLocation()
{
    dataLock.lockForRead();
    Pose retval = amclLocation; 
    dataLock.unlock(); 
    return retval; 	
}

Pose PlayerInterface::getOdomLocation()
{
    dataLock.lockForRead();
    Pose retval = odomLocation; 
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

void PlayerInterface::clearResources()
{
	if(drive)
		delete drive;
	if(vfh)
		delete vfh;
	if(joyStick)
		delete joyStick;
	if(wheelChairCommander)
		delete wheelChairCommander;
	if(map)
		delete map;
	if(ptz)
		delete ptz;
	if(localizer)
		delete localizer;
	if(pc)
		delete pc;	
	if(speechP)
		delete speechP;
}

void PlayerInterface::connectDevices()
{
    if(pc)
    {
		clearResources();
    }
    pc = new PlayerClient(qPrintable(playerHost),playerPort);
    pc->SetDataMode(PLAYER_DATAMODE_PULL);
    //pc->SetDataMode(PLAYER_DATAMODE_PUSH);
	pc->SetReplaceRule(true,-1,-1,-1);
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
		logMsg.append(QString("\n\t\t - Motor Control Interface Engaged Successfully, ID:%1").arg(positionId));	   		 
    }
    for(int i=0; i < lasers.size(); i++)
    {
    	player_pose_t 	lp_pose;
    	lasers[i].lp = new LaserProxy(pc,lasers[i].index);
    	lp_pose = lasers[i].lp->GetPose();
    	lasers[i].pose.p.setX(lp_pose.px);
    	lasers[i].pose.p.setY(lp_pose.py);	    	
    	lasers[i].pose.phi = lp_pose.pa;
    	qDebug("Laser Pose X:%f Y:%f Phi:%f",lasers[i].pose.p.x(),lasers[i].pose.p.y(),lasers[i].pose.phi);	    	
		logMsg.append(QString("\n\t\t - Laser interface:%1 Interface Added Successfully").arg(lasers[i].index));  
    }
    if(mapEnabled)
    {
    	map = new MapProxy(pc,mapId);
		logMsg.append(QString("\n\t\t - Map Interface Engaged Successfully, ID:%1").arg(mapId));			
    }
    if(ptzEnabled)
    {
		ptz = new PtzProxy(pc, ptzId);
		logMsg.append(QString("\n\t\t - Pan Tilt unit initialized Successfully ID:%1").arg(ptzId));			
    }
    if(localizerEnabled)
    {
    	localizer 	  = new LocalizeProxy(pc,0);
    	localizerType = AMCL; 
		logMsg.append(QString("\n\t\t - Localizer Started Successfully ID:%1").arg(0));	    	
    }
    else
    {
    	// no localizer so just set odom as default
    	logMsg.append(QString("\n\t\t - Odom is Used for Localization"));
    	localizerType = ODOM;
    	localized = true;
    }
    if(vfhEnabled)
    {
    	vfh 	= new Position2dProxy(pc,vfhId);
		logMsg.append(QString("\n\t\t - Vfh Started Successfully ID:%1").arg(vfhId));	    	
    }	   
    /* This is temp until the wheelchair interface is added.*/
    joyStickId = 3;
	joyStick = new Position2dProxy(pc,joyStickId);
//	speechP  = new SpeechProxy(pc,0);
//	speechP->Say("I am ready Mate, Heloooooooooooooooooooo");
	sleep(1);
	logMsg.append(QString("\n\t Testing Player Server for Data Read:"));  	
	logMsg.append(QString("\n\t\t - Test Passed, You can read Data from Player Server Now"));
	logMsg.append(QString("\n\t\t - Connection Established"));
	logMsg.append(QString("\n/********************************************************************/"));	
    emit addMsg(0,INFO,logMsg);
	
}
void PlayerInterface::run ()
{
	logMsg.append("\n/********************************************************************/");
	logMsg.append("\nConnecting to Robot Server::");
	logMsg.append(QString("\n\t Connecting to %1:%2 ...").arg(qPrintable(playerHost)).arg(playerPort));
    try
    {
    	connectDevices();
		Timer timer;
	    while(true)
	    {
//	    	qDebug("Loop Time is:%f %d",timer.msecElapsed()); fflush(stdout);
	    	timer.restart();
	    	/* Read Only if new Data is Available*/
//			pc->ReadIfWaiting();
//			if(!pc->Peek(50))
//				continue;
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
		        		vfh->GoTo(vfhGoal.p.x(),vfhGoal.p.y(),vfhGoal.phi);	        		
		        	}
		            getspeed = drive->GetXSpeed();
		            getturnrate = drive->GetYawSpeed();	            
		            ps = drive->GetPose();
		            odomLocation.p.setX(drive->GetXPos());
		            odomLocation.p.setY(drive->GetYPos());
		            odomLocation.phi =  drive->GetYaw();
		            if(joyStick)
		            {
			            joyAxes.setX(joyStick->GetXPos());
			            joyAxes.setY(joyStick->GetYPos());
		            }
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
				    amclLocation.p.setX(localizer->GetHypoth(0).mean.px);
				    amclLocation.p.setY(localizer->GetHypoth(0).mean.py);
				    amclLocation.phi =  localizer->GetHypoth(0).mean.pa;
//				    printf("\nHypo:%d",localizer->GetNumHypoths());
				    //printf("AMCL location %f %f %f",amclLocation.p.x(),amclLocation.p.y(),amclLocation.phi);
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
		        pc->Stop();
		        qDebug("	--->>> Robot Stopped <<<---");
		        // temporary fix, needs more thinking once i finalize things
		        emergencyStopped = true;
		    }
	    	emit newData();
	    }
    }
	catch (PlayerCc::PlayerError e)
  	{
    	std::cerr << e << std::endl;
    	clearResources();
    	this->quit();
    	return;
  	}
}

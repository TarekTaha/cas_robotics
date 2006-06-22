#include "playerinterface.h"

PlayerInterface::PlayerInterface(CommManager *in_comms, QString host, int port):
    comms(in_comms),
    playerHost(host),
    playerPort(port),
    pc(0), 
    driveEnabled(false),
    driveId(0), 
    drive(0), 
    ptzEnabled(false),
    emergencyStopped(false)
{
    qDebug("Robot Ip is: %s", qPrintable(host)); 
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

void PlayerInterface::run ()
{
    qDebug("Connecting to Robot %s port %d ... \n", qPrintable(playerHost), playerPort);
    if(pc)
    {
		delete pc; 
    }
    pc = new PlayerClient(qPrintable(playerHost),playerPort);
    qDebug("... connected"); 
    qDebug("PC is %x", pc); 
    pc->SetFrequency(10); 
    qDebug("Frequency set"); 
    int retryCount=0; 
    if(driveEnabled)
    {
		qDebug("Drive enabled"); 
		if(drive)
		{
		    delete drive;  
		}
	        drive = new PositionProxy(pc,driveId,'a');
    }
    qDebug("Drive started."); 
    for(int i=0; i < MAX_LASERS; i++)
    {
        if(laserEnabled[i])
        {
            laser[i] = new LaserProxy(pc,playerLaserId[0], 'r');  
        }
    }
    if(ptzEnabled)
    {
		ptz = new PtzProxy(pc, ptzId, 'a'); 
    }
    while(pc->Read())
    {
		qWarning("Warning! Could not read from player driver -- sleeping for 1 seconds");
		sleep(1);  
		retryCount++; 
    }
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
            qWarning("Warning! Could not read from player driver - sleeping for 1 seconds."); 
	    	sleep(1); 
	    	retryCount++; 
        }
		qDebug("Emerg stopped: %d", emergencyStopped);
    	if(!emergencyStopped)
    	{
	        if(driveEnabled)
	        {
	            drive->Lock();
	            drive->SetSpeed(speed,turnRate);
	            drive->Unlock();             
	        }
			if(ptzEnabled)
			{
		    	ptz->SetCam(pan,tilt, 1);  
			}
    	}
	    else 
	    {
	        qDebug("Stopping Robot NOW ");
	        if(driveEnabled)
	        {
	            drive->SetSpeed(0,0); 
	            drive->SetMotorState(0);
	        }  
	        qDebug("... Robot Stopped");	   
	    }
    	emit newData();
    }
    
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

void PlayerInterface::enableDrive(int in_driveId)
{
    driveEnabled=true;
    driveId=in_driveId;
}

void PlayerInterface::enablePtz(int in_ptzId)
{
    ptzEnabled=true;
    ptzId=in_ptzId;
}

void PlayerInterface::enableLaser(int laser_id, int in_playerLaserId)
{
    qDebug("Laser enabled"); 
    laserEnabled[laser_id] = true; 
    playerLaserId[laser_id] = in_playerLaserId; 
}

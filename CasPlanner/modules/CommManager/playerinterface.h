#ifndef PLAYERINTERFACE_H
#define PLAYERINTERFACE_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>
#include <libplayerc/playerc.h>

#include <QThread> 
#include <QReadWriteLock>
#include <QTime>
#include <iostream>
#include <QDataStream>

#include "playerinterface.h"
#include "utils.h"
#include "map.h"
#include "timer.h"
#include "statusbar.h"

#define MAX_LASERS 4

using namespace PlayerCc;
using namespace std;

class DeviceType
{
	public:
		DeviceType()
		{
			this->subscribed = false;
		};
		DeviceType(player_devaddr_t addr,QString name)
		{
			this->addr = addr;
			this->driverName = name;
		};
		void setAddress(player_devaddr_t addr)
		{
			this->addr = addr;
		};
		void setName(QString name)
		{
			this->driverName = name;
		};
		player_devaddr_t addr;
  		QString driverName;
  		bool subscribed;
};

////Q_GUI_EXPORT 
//QDataStream &operator<<(QDataStream & out , const DeviceType &dev)
//{
//	out<<dev.subscribed;
//};
//
////Q_GUI_EXPORT 
//QDataStream &operator>>(QDataStream & in , DeviceType &dev)
//{
//	in>>dev.subscribed;
//};

class Laser
{
	public: 
		LaserProxy * lp;
		int index;
		Pose pose;
		~Laser(){};
};

class LaserScan
{
	public:
		QVector<QPointF> points;
		Pose laserPose;
		LaserScan(){};		
		~LaserScan(){};
};
class PlayerInterface: public QThread 
{
Q_OBJECT    
    public:
        PlayerInterface(QString playerHost, int playerPort);
        void stop();
        void run();
        void enableControl(int driveId);
        void setLasers(QVector<Laser> lasers); 
		void enablePtz(int ptzId);
		void enableVfh(int vfhId);
		void enableMap(int mapId);
		void enableLocalizer(int localizerId);
        LaserScan getLaserScan();
        void provideLocation(Pose location);
	    Map provideMap(); 
		void setPtz(double pan, double tilt);
        double getSpeed(); 
        double getTurnRate();
        bool getLocalized();
        Pose getLocation();
        Pose getOdomLocation();
        QVector<DeviceType> * getDevices(QString,int);
        void gotoGoal(Pose);
        void vfhGoto(Pose);
        void setSpeed(double speed);
        void setTurnRate(double turnRate); 
        void setSpeed(double speed, double turnRate); 
        void setLocation(Pose location);
        void emergencyStop();
        void emergencyRelease();
    signals:
        void newData(); 
        void addMsg(int,int,QString);
    private:
        QString playerHost; 
        int playerPort; 
        PlayerClient *pc;
       	playerc_client_t *client;
       	QVector <DeviceType> *devices; 	
        bool ptzEnabled,ctrEnabled,mapEnabled,localizerEnabled,localized,emergencyStopped,
        	 velControl,vfhEnabled;
        int positionId,ptzId,mapId,localizerId,vfhId;
        QVector <Laser> lasers;
        Position2dProxy *drive, *vfh;
        MapProxy *map;
		PtzProxy *ptz;
		LocalizeProxy *localizer;
		Pose location,goal,odom_location,vfhGoal;
		double pan,tilt,pose[3], pose_covar[3];
        LaserScan laserScan;
        double speed,turnRate,getspeed,getturnrate;
        QReadWriteLock dataLock;       
};
#endif 

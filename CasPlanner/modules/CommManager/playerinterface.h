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

#include "utils.h"
#include "map.h"
#include "timer.h"
#include "statusbar.h"
#include "wheelchairproxy.h"

#define MAX_LASERS 4

using namespace PlayerCc;
using namespace std;

//Observations
enum {Up,Down,Right,Left,NoInput};
//Actions- Global Directions
enum {North,South,East,West,Nothing,NE,NW,SE,SW};
//Localizer in Use
enum {GPS,AMCL,ODOM};
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
        ~PlayerInterface();
        void stop();
        void stopRelease();        
        void run();
        void checkForWheelChair();
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
        Pose getAmclLocation();
        Pose getOdomLocation();
        QVector<DeviceType> * getDevices();
        void gotoGoal(Pose);
        void vfhGoto(Pose);
        void setSpeed(double speed);
        void setTurnRate(double turnRate); 
        void setSpeed(double speed, double turnRate); 
		void setOdometry(Pose odom);
        void setLocation(Pose location);
        void connectDevices();
        void clearResources();
		int  getLocalizerType();        
        int  getJoyStickGlobalDir();
        int  getJoyStickDir();
		Pose localizeToPosition(Pose localizerWayPoint);    
    public slots:
     	void terminateMissions();
    signals:
        void newData(); 
        void addMsg(int,int,QString);
    private:
        QString playerHost; 
        QString logMsg;
        int playerPort; 
        int localizerType;
        PlayerClient *pc;
       	playerc_client_t *client;
       	QVector <DeviceType> *devices;
        bool ptzEnabled,ctrEnabled,mapEnabled,localizerEnabled,localized, velControl,vfhEnabled,stopped;
        int positionId,ptzId,mapId,localizerId,vfhId,joyStickId;
        QVector <Laser> lasers;
        Position2dProxy *drive, *vfh, *joyStick;
        WheelChairProxy *wheelChairCommander;
        MapProxy *map;
		PtzProxy *ptz;
		LocalizeProxy *localizer;
		SpeechProxy *speechP;
		Pose amclLocation,goal,odomLocation,vfhGoal;
		QPointF joyAxes;
		double pan,tilt,pose[3], pose_covar[3];
        LaserScan laserScan;
        double speed,turnRate,getspeed,getturnrate;
        QReadWriteLock dataLock;       
};
#endif 

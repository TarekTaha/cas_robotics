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
#include <libplayerinterface/player.h>

#include <QThread> 
#include <QReadWriteLock>
#include <QTime>
#include <iostream>
#include <QDataStream>

#include "comms.h"
#include "utils.h"
#include "map.h"
#include "timer.h"
#include "wheelchairproxy.h"
#include "accelerometer.h"

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
		void setInterfaceName(QString name)
		{
			this->interfaceName= name;
		}
		void setSubscribed(bool sub)
		{
			this->subscribed = sub;
		}
		QString getDriverName()
		{
			return driverName;
		}
		QString getInterfaceName()
		{
			return interfaceName;
		}
		bool isSubscribed()
		{
			return subscribed;
		}
	private:		
		player_devaddr_t addr;
  		QString driverName;
  		QString interfaceName;
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



class PlayerInterface: public Comms
{
Q_OBJECT    
    public:
        PlayerInterface(QString playerHost, int playerPort);
        PlayerInterface();
        ~PlayerInterface();
        void stop();
        void connect2Robot(QString host, int port);
        void stopRelease();        
        void run();
        void checkForWheelChair();
        void enableControl(int driveId);
        void setLasers(QVector<Laser> lasers); 
        void enablePtz(int ptzId);
		void enableJoyStick(int joyId);		
		void enableVfh(int vfhId);
		void enableMap(int mapId);
		void enableSpeech(int speechId);		
		void enableLocalizer(int localizerId);
        LaserScan getLaserScan();
        void provideLocation(Pose location);
	    Map getMap(); 
		void setPtz(double pan, double tilt);
        double getSpeed(); 
        double getTurnRate();
        bool isConnected();
        bool getLocalized();
        Pose getLocation();        
        Pose getAmclLocation();
        Pose getOdomLocation();
        QVector<DeviceType> * getDevices();
        void resetResources();        
        void gotoGoal(Pose);
        void vfhGoto(Pose);
        void setSpeed(double speed); 
        void setTurnRate(double turnRate); 
        void setSpeed(double speed, double turnRate);
        void setSpeechNotification(bool state);   
        bool getSpeechNotificaionStatus();
		void setOdometry(Pose odom);
		void speechSay(QString voiceM);
        void setLocation(Pose location);
        void provideSpeed(double &speed, double &turnRate);
        void connectDevices();
        void clearResources();
        void setCtrEnabled(bool);
        void disconnect();
		int  getLocalizerType();        
        int  getJoyStickGlobalDir();
        int  getJoyStickDir();
		Pose localizeToPosition(Pose localizerWayPoint);    
    signals:
        void newData(); 
        void addMsg(int,int,QString);
    protected:
        QString playerHost; 
        QString logMsg;
        QString voiceMessage;
        int playerPort; 
        int localizerType;
        PlayerClient *pc;
       	playerc_client_t *client;
       	QVector <DeviceType> *devices;
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
        Accelerometer n95Acc;
};
#endif 

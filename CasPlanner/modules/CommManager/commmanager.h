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
#ifndef COMMMANAGER_H
#define COMMMANAGER_H
//#include <libplayerc++/playerc++.h>
//#include <libplayercore/player.h>
#include "playerinterface.h"

class CommManager;
class PlayerInterface;
class Comms;
class PlayGround;

#include "comms.h"
#include "interfaceprovider.h"
#include "robot.h"
#include "statusbar.h"
#include "playground.h"

class CommManager: public Comms, public MapProvider, public LaserProvider, public SpeedProvider ,public LocationProvider
{
    Q_OBJECT
    public:
        CommManager(Robot *,PlayGround * playG);
        ~CommManager(); 
        virtual int readConfigs(ConfigFile *cf,int secId);
        virtual int start(); 
        virtual int stop();
        virtual LaserScan getLaserScan();
        virtual double getSpeed(); 
        virtual double getTurnRate();
        virtual int    getJoyStickGlobalDir();
        virtual int    getLocalizerType();
        virtual int    getJoyStickDir(); 
        virtual Pose   getLocation();               
        virtual Pose   getAmclLocation();
        virtual Pose   getOdomLocation();
        virtual void   gotoGoal(Pose);
        virtual void   vfhGoto(Pose);
        virtual void   setSpeed(double speed);
        virtual void   setPtz(double pan, double tilt);
        virtual void   setTurnRate(double turnRate);
        virtual void   setOdometry(Pose odom);
	    virtual Map    provideMap(); 
	    virtual void   provideSpeed(double &speed, double &turnRate);
	    virtual void   provideLocation(Pose & location);
	    virtual bool   getLocalized();
		virtual QVector<DeviceType> * getDevices();
		bool connected;
    public slots: 
        virtual void setSpeed(double speed, double turnRate); 
        virtual void setLocation(Pose location);
        virtual void emergencyStop();
        virtual void emergencyRelease();
    signals:
	    void dataUpdated();
	    void addMsg(int,int, QString);
    protected:
        // Player stuff 
        PlayerInterface *player;
       	QVector <Laser> lasers;
        Robot * robot;
       	PlayGround *playGround;
};

#endif


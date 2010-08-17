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
#ifndef COMMS_H
#define COMMS_H
#include <QObject>
#include <QString> 
#include <QStringList>
#include <QVector>
#include <QPointF>
#include <QThread>
#include "configfile.h"
#include "utils.h"
#include "map.h"
#include "interfaceprovider.h"
#include "casplannerexception.h"
#include "statusbar.h"

//Observations
enum {Up,Down,Right,Left,NoInput};

//Actions- Global Directions
enum {North,South,East,West,Nothing,NE,NW,SE,SW};

//Localizer in Use
enum {GPS,AMCL,ODOM};

class Comms: public QThread, public LocationProvider, public MapProvider, public SpeedProvider, public LaserProvider 
{
    Q_OBJECT 
    public:
        virtual int readConfigs(ConfigFile *cf,int secId)=0;
        virtual void connect2Robot(QString playerHost, int playerPort)=0;
		virtual QString getName()
		{
		    return name; 
		}
    Q_SIGNALS:
        void newData();
		void addMsg(int,int,QString); 
    public Q_SLOTS:
        virtual void stop()=0;
        virtual void stopRelease()=0;
        virtual void disconnect()=0; 
    protected:
		bool startConnected,activateControl,ptzEnabled,occMapEnabled,localizerEnabled,laserEnabled
			 ,vfhEnabled,speechEnabled,connected, joyStickEnabled, ctrEnabled, mapEnabled, localized 
			 ,velControl, stopped;
    	QString name,playerIp; 
        int positionId, playerPort,positionControlId,ptzId,mapId,localizerId,vfhId,speechId,joyStickId;
};

#endif 


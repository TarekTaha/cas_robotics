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
#include "configfile.h"
/* 
 * This class specifies the must have operation that every
 * robot should have in their comminication management.
 */
class Comms: public QObject 
{
    Q_OBJECT 
    public:
        virtual int readConfigs(ConfigFile *cf,int secId)=0;
        virtual int start()=0;
		virtual QString getName()
		{
		    return name; 
		}
    signals:
        void newData();
		void statusMsg(int,int,QString); 
    public slots:
    	virtual void stopMotors()=0;
        virtual void stop()=0;
        virtual void stopRelease()=0;
        virtual void disconnect()=0; 
    protected:
		bool startConnected,activateControl,ptzEnabled,occMapEnabled,localizerEnabled,laserEnabled
			 ,vfhEnabled,speechEnabled;
    	QString name,playerIp; 
        int playerPort,positionControlId,ptzId,mapId,localizerId,vfhId,speechId,joyStickId;
};

#endif 


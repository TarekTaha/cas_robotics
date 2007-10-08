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
#ifndef PLAYGROUND_H_
#define PLAYGROUND_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QVector>
#include <QStatusBar> 
#include "navigationtab.h"
#include "mapviewer.h"
#include "robotmanager.h"
#include "configfile.h"
#include "statusbar.h"

class Navigator;
class NavContainer;
class PlanningManager;
class MapViewer;

class PlayGround: public QObject
{
	Q_OBJECT
	public:
		PlayGround();
		PlayGround(QStringList configFiles,QStatusBar *in_statusBar);
		virtual ~PlayGround();
		int renderingMethod;
		NavContainer *navCon;
		MapViewer    *mapViewer;
		MapManager   *mapManager;
		StatusLogger *statusLogger;	
		QVector <RobotManager* > robotPlatforms;
		int  setNavContainer(NavContainer* con);
	public slots:	
		void startRobotsComm();
		void stopRobots();
		void loadMap(QString name,float res,bool negate, Pose p);
		void addMsg(int id,int type,QString msg);				
    signals:
	    void mapUpdated(Map * mapData);		
};

#endif /*PLAYGROUND_H_*/

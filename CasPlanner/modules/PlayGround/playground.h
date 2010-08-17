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

#include <QVector>
#include <QStatusBar>
#include <QString>
#include "xmlparser.h"
#include "logger.h"
#include "utils.h"

class Navigator;
class PlanningManager;
class MapViewer;
class SocialPlanner;
class StatusLogger;
class MapManager;
class RobotManager;
class Map;
class PlayGround: public QObject
{
    Q_OBJECT
public:
    PlayGround();
    PlayGround(QStringList configFiles,QStatusBar *in_statusBar);
    virtual ~PlayGround();
    int renderingMethod;
    MapViewer    *mapViewer;
    MapManager   *mapManager;
    StatusLogger *statusLogger;
    QVector <RobotManager* > robotPlatforms;
    RobotManager *activeRobot;
    void setMapViewer(MapViewer *);
public Q_SLOTS:
        void startRobotsComm();
        void stopRobots();
        void loadMap(QString name,float res,bool negate, Pose p);
        void addMsg(int id,int type,QString msg);
Q_SIGNALS:
    void mapUpdated(Map * mapData);
};

#endif /*PLAYGROUND_H_*/

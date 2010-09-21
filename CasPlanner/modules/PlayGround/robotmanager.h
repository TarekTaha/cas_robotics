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
#ifndef ROBOTMANAGER_H
#define ROBOTMANAGER_H

#include <QObject>

class Navigator;
class IntentionRecognizer;
class CommManager;
class PlayGround;
class ConfigFile;
class PlanningManager;
class Robot;
class SocialPlanner;
class Map;

enum{FORCE_FIELD,VFH,CONFIG_SPACE,NO_AVOID,ND};
enum{MINIMAL_INPUT,CONTINIOUS_INPUT};

class RobotManager : public QObject
{
    Q_OBJECT
    public:
        RobotManager();
        RobotManager(PlayGround *playG,ConfigFile *cf,int secId);
        ~RobotManager();
        bool hasPath();
        bool isConnected();
        bool isNavigating();
        bool isNavigationPaused();
        void readRobotConfigs(ConfigFile *cf,int secId);
        void readCommManagerConfigs(ConfigFile *cf,int secId);
        void readPlannerConfigs(ConfigFile *cf);
        void readNavigatorConfigs(ConfigFile *cf);
        void setupSocialPlanner();
        void setPauseNavigation(bool);
        void start();
        void stop();
        void stopNavigating();
        void startPlanner();
        void startNavigator();
        void startNavigating();
        void startComms();
        void startIntentionRecognizer();
        PlayGround 	*playGround;
        CommManager     *commManager;
        PlanningManager *planningManager;
        Navigator       *navigator;
        IntentionRecognizer *intentionRecognizer;
        Robot           *robot;
        SocialPlanner   *socialPlanner;
        public
    Q_SLOTS:
                void updateMap(Map * mapData);
    Q_SIGNALS:
        void addMsg(int,int,QString);
};

#endif


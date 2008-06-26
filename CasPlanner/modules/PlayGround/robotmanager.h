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

#include "commmanager.h"
#include <QObject>
#include "planningmanager.h"
#include "navigator.h"
#include "configfile.h"
#include "robot.h"
#include "playground.h"
#include "IntentionRecognizer.h"

class Navigator;
class NavContainer;
class PlanningManager;
class MapViewer;
class PlayGround;
class IntentionRecognizer;

enum{FORCE_FIELD,VFH,CONFIG_SPACE,NO_AVOID,ND};
enum{MINIMAL_INPUT,CONTINIOUS_INPUT};

class RobotManager : public QObject//public CommManager, public PlanningManager ,public Navigator
{
	Q_OBJECT
    public:
    	RobotManager();
		RobotManager(PlayGround *playG,ConfigFile *cf,int secId);
		~RobotManager();
		int readRobotConfigs(ConfigFile *cf,int secId);
		int readCommManagerConfigs(ConfigFile *cf,int secId);
		int readPlannerConfigs(ConfigFile *cf);
		int readNavigatorConfigs(ConfigFile *cf);
//		int setupSocialPlanner();
		int start();
		int stop();
		int startPlanner();
		int startNavigator();
		int startComms();
		int startIntentionRecognizer();
		PlayGround 		*playGround;
		CommManager     *commManager;
		PlanningManager *planningManager;
		Navigator       *navigator;
		IntentionRecognizer *intentionRecognizer;
		Robot           *robot;
//		CasPlanner::SocialPlanner   *socialPlanner;
		bool notPaused,notFollowing;
	public 
	slots:
		void updateMap(Map * mapData);
	signals:
		void addMsg(int,int,QString);
};

#endif


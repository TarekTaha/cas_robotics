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
#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>
#include "commmanager.h"
#include "planningmanager.h"
#include "navigator.h"
#include "configfile.h"
#include "playground.h"
#include "IntentionRecognizer.h"
#include "socialplanner.h"

class Navigator;
class IntentionRecognizer;
class CommManager;


enum{FORCE_FIELD,VFH,CONFIG_SPACE,NO_AVOID,ND};
enum{MINIMAL_INPUT,CONTINIOUS_INPUT};

class RobotManager : public QObject
{
	Q_OBJECT
    public:
    	RobotManager();
		RobotManager(PlayGround *playG,ConfigFile *cf,int secId);
		~RobotManager();
		void readRobotConfigs(ConfigFile *cf,int secId);
		void readCommManagerConfigs(ConfigFile *cf,int secId);
		void readPlannerConfigs(ConfigFile *cf);
		void readNavigatorConfigs(ConfigFile *cf);
		void setupSocialPlanner();
		void start();
		void stop();
		void startPlanner();
		void startNavigator();
		void startComms();
		void startIntentionRecognizer();
		PlayGround 		*playGround;
		CommManager     *commManager;
		PlanningManager *planningManager;
		Navigator       *navigator;
		IntentionRecognizer *intentionRecognizer;
		Robot           *robot;
		SocialPlanner   *socialPlanner;
		bool notPaused,notFollowing;
	public 
	slots:
		void updateMap(Map * mapData);
	signals:
		void addMsg(int,int,QString);
};

#endif


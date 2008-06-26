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
#ifndef INTENTIONRECOGNIZER_H_
#define INTENTIONRECOGNIZER_H_

#include "playground.h"
#include <QThread> 
#include <QReadWriteLock>
#include <QTime>
#include <QDate>
#include <iostream>
#include <QDataStream>
#include "robot.h"
#include "MatrixUtils.h"
#include "BoundPairExec.h"
#include "zmdpMainConfig.h"
#include "activityLogger.h"
#include "Pomdp.h"
#include "socialplanner.h"

//using namespace std;
class SocialPlanner;

class IntentionRecognizer: public QThread
{
public:
	~IntentionRecognizer();
	void InitializePOMDP();
	IntentionRecognizer(PlayGround * playG,RobotManager *robotManager);
	void followActionToNextState();
	void navigateToWayPoint(Pose start,Pose dest);
	void run();
	void resetBelief();
	bool currentStateIsDestination();
	bool runRecognition;
	bool useNavigator,beliefInitialized;
	int nextState;
	int numDestinations;
	int numStates;
	int  getInteractionStrategy();
	void setInteractionStrategy(int strategy);
	QVector <double> destBelief;	
	Pose currentState,currentPose,goToState,oldGoToState;
	ActivityLogger *activityLogger;
	int  lastObs,observation,action,spatialState,oldSpatialState;
	SocialPlanner *socialPlanner;
private:
	dvector initialBeliefD;
	int interactionStrategy;
	bool actionAmbiguity;
	string pomdpFileName, policyFileName;
	zmdp::ZMDPConfig* config;
	zmdp::BoundPairExec* em;
	PlayGround * playGround;
	RobotManager * robotManager;
	QReadWriteLock dataLock;
	Node * path;
	FILE *file;
	FILE *odom;
};

#endif /*INTENTIONRECOGNIZER_H_*/

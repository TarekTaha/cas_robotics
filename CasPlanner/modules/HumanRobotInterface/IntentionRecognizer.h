#ifndef INTENTIONRECOGNIZER_H_
#define INTENTIONRECOGNIZER_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>
#include <libplayerc/playerc.h>

#include <QThread> 
#include <QReadWriteLock>
#include <QTime>
#include <iostream>
#include <QDataStream>
#include "robot.h"
#include "playground.h"

#include "MatrixUtils.h"
#include "BoundPairExec.h"
#include "zmdpMainConfig.h"
#include "activityLogger.h"

#include "Pomdp.h"
using namespace std;
using namespace zmdp;

class IntentionRecognizer: public QThread
{
public:
	~IntentionRecognizer();
	void InitializePOMDP();
	IntentionRecognizer(PlayGround * playG,RobotManager *robotManager);
	void followActionToNextState();
	void run();
	bool runRecognition;
	bool beliefInitialized;
	int nextState;
	int numDestinations;
	int numStates;
	QVector <double> destBelief;	
	Pose location,goToState,oldGoToState;
	ActivityLogger activityLogger;
	int  observation,action,spatialState,oldSpatialState;
private:
	string pomdpFileName, policyFileName;
	ZMDPConfig* config;
	BoundPairExec* em;
	PlayGround * playGround;
	RobotManager * robotManager;
	QReadWriteLock dataLock; 
};

#endif /*INTENTIONRECOGNIZER_H_*/

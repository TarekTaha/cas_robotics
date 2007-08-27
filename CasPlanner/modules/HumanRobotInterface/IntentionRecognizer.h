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
	void navigateToWayPoint(Pose start,Pose dest);
	void run();
	void resetBelief();
	bool currentStateIsDestination();
	bool runRecognition;
	bool useNavigator,beliefInitialized;
	int nextState;
	int numDestinations;
	int numStates;
	QVector <double> destBelief;	
	Pose currentState,location,goToState,oldGoToState;
	ActivityLogger activityLogger;
	int  observation,action,spatialState,oldSpatialState;
private:
	dvector initialBeliefD;
	string pomdpFileName, policyFileName;
	ZMDPConfig* config;
	BoundPairExec* em;
	PlayGround * playGround;
	RobotManager * robotManager;
	QReadWriteLock dataLock;
	Node * path;	 
};

#endif /*INTENTIONRECOGNIZER_H_*/

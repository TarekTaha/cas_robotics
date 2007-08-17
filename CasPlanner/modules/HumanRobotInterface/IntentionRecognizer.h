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

#include "Pomdp.h"
using namespace std;
using namespace zmdp;

class IntentionRecognizer: public QThread
{
public:
	IntentionRecognizer();
	~IntentionRecognizer();
	void InitializePOMDP();
	IntentionRecognizer(PlayGround * playG,RobotManager *robotManager);
	void run();
	bool runRecognition;
	bool beliefInitialized;
	int nextState;
	int numDestinations;
	int numStates;
	QVector <double> destBelief;	
private:
	string pomdpFileName, policyFileName;
	ZMDPConfig* config;
	BoundPairExec* em;
	PlayGround * playGround;
	RobotManager * robotManager;
};

#endif /*INTENTIONRECOGNIZER_H_*/

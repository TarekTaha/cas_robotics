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
#include "IntentionRecognizer.h"

IntentionRecognizer::IntentionRecognizer(PlayGround * playG, RobotManager *rManager):
runRecognition(false),
useNavigator(false),
beliefInitialized(false),
destBelief(playG->mapManager->mapSkeleton.numDestinations,0),
goToState(-1,-1,-1),
oldGoToState(0,0,0),
lastObs(4),
observation(4),
action(4),
spatialState(-1),
oldSpatialState(-2),
interactionStrategy(CONTINIOUS_INPUT),
//interactionStrategy(MINIMAL_INPUT),
actionAmbiguity(false),
playGround(playG),
robotManager(rManager),
path(NULL)
{
	InitializePOMDP();
	if (!playGround)
	{
		perror("\nIR: PlayGround Not Passed Correctly");
	}
	numStates = playGround->mapManager->mapSkeleton.numStates;
	numDestinations = playGround->mapManager->mapSkeleton.numDestinations;
	activityLogger = new ActivityLogger;
}

IntentionRecognizer::~IntentionRecognizer()
{
	delete activityLogger;
	fclose(file);
	fclose(odom);
}

void IntentionRecognizer::InitializePOMDP()
{
  	MatrixUtils::init_matrix_utils();
  	//pomdpFileName  = "modules/PomdpModels/minimal_interaction.pomdp";
 	pomdpFileName  = "modules/PomdpModels/paperexperiment.pomdp";
  	//policyFileName = "modules/PomdpModels/mod11.policy";
 	policyFileName = "modules/PomdpModels/6dest.policy";
  	//policyFileName = "modules/PomdpModels/minimal_interaction.policy";
  	config = new ZMDPConfig();
  	config->readFromFile("modules/PomdpCore/zmdp.conf");
  	config->setString("policyOutputFile", "none");
  	em = new BoundPairExec();
  	printf("initializing\n");
  	em->initReadFiles(pomdpFileName,policyFileName, *config);
}

void IntentionRecognizer::setInteractionStrategy(int strategy)
{
	if(strategy != CONTINIOUS_INPUT && strategy != MINIMAL_INPUT)
	{
		qDebug("\n Undefined Interaction Method");
		return;
	}
	this->interactionStrategy = strategy;
	printf("\n Strategy:%d",strategy);
	return;
}

int IntentionRecognizer::getInteractionStrategy()
{
	return this->interactionStrategy;
}

void IntentionRecognizer::navigateToWayPoint(Pose startLoc,Pose destLoc)
{
	robotManager->planningManager->setStart(startLoc);
	robotManager->planningManager->setEnd(destLoc);
	path = robotManager->planningManager->findPath(METRIC);	

	robotManager->navigator->setPause(false);
	robotManager->navigator->setObstAvoidAlgo(ND);
	
	printf("\n---IR:Commanded the Navigator");fflush(stdout);
	if(path)
	{
		if(!robotManager->navigator->isRunning())
		{
			printf("\n---IR:Starting The Navigator");fflush(stdout);
			robotManager->navigator->setPath(path);
			robotManager->navigator->start();
		}
		else
		{
			robotManager->navigator->StopNavigating();
			msleep(200); // Wait until the Thread Terminates
			printf("\n---IR:Giving New Path to Navigator");fflush(stdout);			
			robotManager->navigator->setPath(path);
			robotManager->navigator->start();						
		}
	}
	else
	{
		printf("\n---IR:Path not Found, attempting to use local navigator VFH");fflush(stdout);
		if(robotManager->navigator->isRunning())
		{
			printf("\n---IR:Stopping Previous Navigator.");fflush(stdout);			
			robotManager->navigator->StopNavigating();
			msleep(200); // Wait until the Thread Terminates
			robotManager->navigator->quit();
		}
		robotManager->navigator->start();
		robotManager->commManager->vfhGoto(destLoc);
	}
}

void IntentionRecognizer::followActionToNextState()
{
    /* 
     * Find where the currtent action will take us from the Transition Matrix
     * stored in the POMDP model.
     */ 
    double maxTrans = 0;
	for (int sp=0; sp < ((Pomdp*)em->mdp)->getBeliefSize(); sp++) 
    {
    	if(((Pomdp*)em->mdp)->T[action](spatialState,sp) >= maxTrans)
    	{
//    		printf("\nIndex=%d %5.3f ",sp,((Pomdp*)em->mdp)->T[action](spatialState,sp));
			maxTrans = ((Pomdp*)em->mdp)->T[action](spatialState,sp);
			nextState = sp;
    	}
    }
    fprintf(file,"%d\n",nextState);
    if(maxTrans==0)
    {
    	printf("\nIR: This action is not applicable to this state."); fflush(stdout);
    	return;
    }
  
	goToState.p.setX(playGround->mapManager->mapSkeleton.verticies[nextState].location.x());
	goToState.p.setY(playGround->mapManager->mapSkeleton.verticies[nextState].location.y());
	
//	currentState.p.setX(l.x());
//	currentState.p.setY(currentPose.y());
//	currentState.phi = currentPose.phi();
	/* 
	 * Set Final Orientation to the direction of Motion 
	 */
	switch(action)
	{
		case North:
			goToState.phi = DTOR(90);
			if(robotManager->commManager->getSpeechNotificaionStatus())
				robotManager->commManager->speechSay(QString("Going North"));
			break;
		case South:
			goToState.phi = DTOR(270);
			if(robotManager->commManager->getSpeechNotificaionStatus())
				robotManager->commManager->speechSay(QString("Going South"));			
			break;
		case East:
			goToState.phi = DTOR(0);
			if(robotManager->commManager->getSpeechNotificaionStatus())
				robotManager->commManager->speechSay(QString("Going East"));			
			break;
		case West:
			goToState.phi = DTOR(180);
			if(robotManager->commManager->getSpeechNotificaionStatus())
				robotManager->commManager->speechSay(QString("Going West"));			
			break;
		case Nothing:
			goToState.phi = DTOR(0);
			if(robotManager->commManager->getSpeechNotificaionStatus())
				robotManager->commManager->speechSay(QString("Staying in Location"));			
			break;
		default:
			goToState.phi = DTOR(0);
	}
//	if( (oldGoToState!=goToState && observation!=NoInput) || !beliefInitialized)
	if( (oldGoToState!=goToState) || !beliefInitialized)
	{
		printf("\n Going to State:%d",nextState);
		printf(" oldGoto X=%f, Y=%f, GoTo X=%f, Y=%f Action=%d Obs=%d",oldGoToState.p.x(),oldGoToState.p.y(),goToState.p.x(),goToState.p.y(),action,observation);

		if(robotManager->commManager)
	  		robotManager->commManager->stopRelease();

		if(!useNavigator)
			robotManager->commManager->vfhGoto(goToState);
		else
			navigateToWayPoint(currentPose,goToState);
			
		oldGoToState= goToState;
	}
}

void IntentionRecognizer::resetBelief()
{
  	belief_vector b;
  	initialBeliefD.resize(((Pomdp*)em->mdp)->getBeliefSize());
  	if(robotManager->commManager)
  		robotManager->commManager->stop();
  	/*
  	 * Distribute belief Evenly accross destinations
  	 */ 
  	for(int i=0;i < numDestinations;i++)
  	{
  		destBelief[i] = initialBeliefD((numStates*i) + spatialState) = 1/float(numDestinations);
  	}
//  	destBelief[0] = initialBeliefD((numStates*0) + spatialState) = 0;
//  	destBelief[1] = initialBeliefD((numStates*1) + spatialState) = 0.25;
//  	destBelief[2] = initialBeliefD((numStates*2) + spatialState) = 0;
//  	destBelief[3] = initialBeliefD((numStates*3) + spatialState) = 0.75;
  	copy(b, initialBeliefD);
  	oldSpatialState = -2;
  	em->setBelief(b);
  	printf("\n Belief Reset !!!");
}

bool IntentionRecognizer::currentStateIsDestination()
{
	for(int i=0; i<playGround->mapManager->mapSkeleton.destIndexes.size();i++)
	{
		if(spatialState == playGround->mapManager->mapSkeleton.destIndexes[i] )
			return true;
	}
	return false;
}

void IntentionRecognizer::run()
{
	QDate date;
	QTime time;
	date = date.currentDate();
	time = time.currentTime();
	QString suffex=QString("-%1%2%3%4%5.txt").arg(time.minute()).arg(time.hour()).arg(date.day()).arg(date.month()).arg(date.year());
	file=fopen(qPrintable(QString("logs/irLog%1").arg(suffex)),"wb");
	odom=fopen(qPrintable(QString("logs/odom%1").arg(suffex)),"wb");
	fprintf(file,"# Values: State, Observation, Belief Dest1 ...n, Action, Next State\n");	
	fprintf(odom,"# Values: X, Y, Theta, Spatial State \n");
	bool justStarted = true;
	while(runRecognition)
	{
		msleep(50);
		if(!robotManager->commManager)
		{
			qDebug("\t - (IR): Communication Manager Not Initialized");
			continue;
		}
		if(!robotManager->commManager->connected)
		{
			qDebug("\t - (IR): Your not Connected to the Robot, Connect First");
			continue;		
		}
		while(!robotManager->commManager->getLocalized())
		{
			currentPose = robotManager->commManager->getLocation();
			qDebug("\n---IR:NO Accurate Estimation yet, best current is: x:%f y:%f phi:%f",currentPose.p.x(),currentPose.p.y(),RTOD(currentPose.phi));
		  	if(robotManager->commManager)
		  		robotManager->commManager->stop();
			usleep(300000);
		}
		currentPose = robotManager->commManager->getLocation();
		spatialState = playGround->mapManager->mapSkeleton.getCurrentSpatialState(currentPose);
		fprintf(odom,"%.3f %.3f %.3f %d\n",currentPose.p.x(),currentPose.p.y(),currentPose.phi,spatialState);
		observation = robotManager->commManager->getJoyStickGlobalDir();
		if(!beliefInitialized)
		{
			/*
			 *  When we first connect the readings might not be correct
			 *  so we need to pose to let the commManager properly initialize then 
			 *  request the odom correctly.
			 */
			sleep(1);
			currentPose = robotManager->commManager->getLocation();
			spatialState = playGround->mapManager->mapSkeleton.getCurrentSpatialState(currentPose);			
			resetBelief();
		  	beliefInitialized = true;
		}
  
		/* Take observations and update Beliefs only in discrete states*/
		double dist = playGround->mapManager->mapSkeleton.getDist2SpatialState(currentPose,spatialState);
		if(oldSpatialState != spatialState)
		{
//			if(!(dist<=1.0 || justStarted))
//				continue;
			/* We are now in action new State so Save it as visited */
		    // Get an Observation
			lastObs = observation = robotManager->commManager->getJoyStickGlobalDir();
		    /* 
		     * Get an observation and if it's directional(not NoInput) then don't
		     * take any more observations from this state.
		     */
		    
			if(observation == NoInput && interactionStrategy == CONTINIOUS_INPUT) 
		    	continue;
			
			if(interactionStrategy == MINIMAL_INPUT)
			{
				if(observation != NoInput)
					actionAmbiguity = false;
				else if (actionAmbiguity)	//observation == NoInput
					continue;					
			}

			if (currentStateIsDestination())//em->getStateIsTerminal())
		    {
		    	resetBelief();
				printf("\n[belief is terminal, Resetting Belief]\n");
				activityLogger->addState(spatialState,NoInput);
				activityLogger->startNewTask();
		    }

		    belief_vector prevB = em->currentState;
		    QVector <int> possibleActions;		    
		    possibleActions.clear();
		    int maxBeliefAction=-1;
		    double currentMaxBelief=0;
		    if(interactionStrategy == MINIMAL_INPUT && observation == NoInput)
		    {
//		    	belief_vector thisMaxBelief = em->currentState;
//		    	currentMaxBelief = 0;
//		    	for(unsigned int j=0; j < thisMaxBelief.size();j++)
//		    	{
//		    		if(thisMaxBelief(j)>currentMaxBelief)
//		    			currentMaxBelief = thisMaxBelief(j);
//		    	}
		    	currentMaxBelief = 0;
		    	for(int j=0; j < destBelief.size();j++)
		    	{
		    		if(destBelief[j]>currentMaxBelief)
		    			currentMaxBelief = destBelief[j];
		    	}
		    	actionAmbiguity = false;
		    	double maxBelief=0;
			    for(int whatIfObs=0;whatIfObs<5;whatIfObs++)
			    {
			    	// See What the result of obtaining this observation would be
			    	em->advanceToNextState(action,whatIfObs);
			    	/* 
			    	 * Add the action to be excuted to the set of actions if
			    	 * it's not already in the list 
			    	 */
			    	belief_vector thisMaxBelief = em->currentState;
			    	maxBelief = 0;
			    	for(unsigned int j=0; j < thisMaxBelief.size();j++)
			    	{
			    		if(thisMaxBelief(j) > 0.5)
			    		{
			    			maxBelief = thisMaxBelief(j);
			    			break;
			    		}
			    		if(thisMaxBelief(j)>maxBelief)
			    			maxBelief = thisMaxBelief(j);
			    	}
			    	if(possibleActions.indexOf(em->chooseAction())==-1 && maxBelief>currentMaxBelief)
			    	{
			    		possibleActions.push_back(em->chooseAction());
			    		maxBeliefAction = whatIfObs;
			    		if(possibleActions.size()>1)
					    {
					    	printf("\n I need an input, i don't know where you are going !!!");fflush(stdout);
					    	actionAmbiguity = true;
					    	em->setBelief(prevB);
					    	break;
					    }
			    	}
			    	// Get Back to original Belief and try another observation
			    	em->setBelief(prevB);
			    }
			    for(int i =0;i<possibleActions.size();i++)
			    {
			    	printf("\nPossible action:[%d]",possibleActions[i]); fflush(stdout);
			    }
		    }
		    if (actionAmbiguity)
		    	continue;
		    if(possibleActions.size()> 0)
		    	observation = maxBeliefAction;
		    oldSpatialState = spatialState;
		    em->advanceToNextState(action,observation);
		    belief_vector newB = em->currentState;
		    
		    action = em->chooseAction();
			fprintf(file,"%d ",action);
		    printf("\n New Chosen Action is:%d",action);			

			activityLogger->addState(spatialState,observation);
			fprintf(file,"%d %d ",spatialState,observation);
			
		    QVector<double> max(numDestinations,0.0);
		    int index=0;
			for(unsigned int j=0; j < newB.size();j++)
			{
				index = (int)(j/numStates);
				if( newB(j))//&& (newB(j) > max[index]) )
				{
					dataLock.lockForWrite();
						destBelief[index] = max[index] = (newB(j) + max[index]);
					dataLock.unlock();
				}
			}
			for(int i=0;i<numDestinations;i++)
			{
				printf("\n Belief is now Updated to Dest:%d with %f",i,destBelief[i]);				
				fprintf(file,"%.5f ",destBelief[i]);
			}
			followActionToNextState();
			justStarted = false;
		}
	}	
}

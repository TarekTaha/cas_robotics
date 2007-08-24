#include "IntentionRecognizer.h"

IntentionRecognizer::IntentionRecognizer(PlayGround * playG, RobotManager *rManager):
runRecognition(false),
useNavigator(true),
beliefInitialized(false),
destBelief(numDestinations,0),
goToState(-1,-1,-1),
oldGoToState(0,0,0),
observation(4),
action(4),
spatialState(-1),
oldSpatialState(-2),
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
}

IntentionRecognizer::~IntentionRecognizer()
{
	
}

void IntentionRecognizer::InitializePOMDP()
{
  MatrixUtils::init_matrix_utils();
  pomdpFileName  = "/home/BlackCoder/Desktop/paperexperiment.pomdp";
  policyFileName = "/home/BlackCoder/Desktop/out.policy";
  config = new ZMDPConfig();
  config->readFromFile("modules/PomdpCore/zmdp.conf");
  config->setString("policyOutputFile", "none");
  em = new BoundPairExec();
  printf("initializing\n");
  em->initReadFiles(pomdpFileName,policyFileName, *config);	
}

void IntentionRecognizer::navigateToWayPoint(Pose startLoc,Pose destLoc)
{
	robotManager->planningManager->setStart(startLoc);
	robotManager->planningManager->setEnd(destLoc);
	path = robotManager->planningManager->findPath(METRIC);	
	
	robotManager->navigator->setPause(false);
	

		
	robotManager->navigator->setObstAvoidAlgo(NO_AVOID);
	
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
    	if(((Pomdp*)em->mdp)->T[action](spatialState,sp) > maxTrans)
    	{
//			printf("Index=%d %5.3f ",sp,((Pomdp*)em->mdp)->T[action](spatialState,sp));
			maxTrans = ((Pomdp*)em->mdp)->T[action](spatialState,sp);
			nextState = sp;
    	}
    }
    if(maxTrans==0)
    {
    	printf("\nIR: This action is not applicable to this state."); fflush(stdout);
    	return;
    }
    
	goToState.p.setX(playGround->mapManager->mapSkeleton.verticies[nextState].location.x());
	goToState.p.setY(playGround->mapManager->mapSkeleton.verticies[nextState].location.y());
	
	currentState.p.setX(playGround->mapManager->mapSkeleton.verticies[spatialState].location.x());
	currentState.p.setY(playGround->mapManager->mapSkeleton.verticies[spatialState].location.y());

	/* 
	 * Set Final Orientation to the direction of Motion 
	 */
	switch(action)
	{
		case North:
			goToState.phi = DTOR(90);
			break;
		case South:
			goToState.phi = DTOR(270);
			break;
		case East:
			goToState.phi = DTOR(0);
			break;
		case West:
			goToState.phi = DTOR(180);
			break;
		case Nothing:
			goToState.phi = DTOR(0);
			break;
		default:
			goToState.phi = DTOR(0);
	}
	if( (oldGoToState!=goToState && observation!=NoInput) || !beliefInitialized)
	{
		printf("\n Going to State:%d",nextState);
		printf("\noldGoto X=%f, Y=%f, GoTo X=%f, Y=%f Action=%d Obs=%d",oldGoToState.p.x(),oldGoToState.p.y(),goToState.p.x(),goToState.p.y(),action,observation);
		if(!useNavigator)
			robotManager->commManager->vfhGoto(goToState);
		else
			navigateToWayPoint(currentState,goToState);
			
		oldGoToState= goToState;
	}
}

void IntentionRecognizer::run()
{
//	bool startState = true;
	while(runRecognition)
	{
		msleep(50);
		if(!robotManager->commManager)
		{
//			qDebug("\t - (IR): Communication Manager Not Initialized");
			continue;
		}
		if(!robotManager->commManager->connected)
		{
//			qDebug("\t - (IR): Your not Connected to the Robot, Connect First");
			continue;		
		}
		
		location = robotManager->commManager->getOdomLocation();
		spatialState = playGround->mapManager->mapSkeleton.getCurrentSpatialState(location);
		observation = robotManager->commManager->getJoyStickGlobalDir();
				
		if(!beliefInitialized)
		{
			/*
			 *  When we first connect the readings might not be correct
			 *  so we need to pose to let the commManager properly initialize then 
			 *  request the odom correctly.
			 */
			sleep(1);
			location = robotManager->commManager->getOdomLocation();
			spatialState = playGround->mapManager->mapSkeleton.getCurrentSpatialState(location);			
		  	belief_vector b;
		  	dvector initialBeliefD;
		  	initialBeliefD.resize(((Pomdp*)em->mdp)->getBeliefSize());
		  	/*
		  	 * Distribute belief Evenly accross destinations
		  	 */ 
		  	for(int i=0;i < numDestinations;i++)
		  	{
		  		destBelief[i] = initialBeliefD((numStates*i) + spatialState) = 1/float(numDestinations);
		  	}
		  	copy(b, initialBeliefD);
		  	em->setBelief(b);		
		  	/* Chose Initial Action */
		  	action = em->chooseAction();
		  	followActionToNextState();
		  	beliefInitialized = true;
		}
//		printf("\nJoyStick Global Direction=%d Current Location X=%f, Y=%f CurrentState=%d",robotManager->commManager->getJoyStickGlobalDir(),location.p.x(),location.p.y(),spatialState);   
		/* Take observations and update Beliefs only in discrete states*/
		if(oldSpatialState != spatialState)
		{
			/* We are now in action new State so Save it as visited */
		    // Get an Observation
		    observation = robotManager->commManager->getJoyStickGlobalDir();
		    /* 
		     * Get an observation and if it's directional(not NoInput) then don't
		     * take any more observations from this state.
		     */
		    if(observation==NoInput)
		    {
		    	continue;
		    }
//		    if(startState)
//		    {
//			  	action = em->chooseAction();
//			  	followActionToNextState();		
//			  	startState = false;    	
//			  	continue;
//		    }
		    
		    oldSpatialState = spatialState;
		    em->advanceToNextState(action,observation);
		    belief_vector newB = em->currentState;
			activityLogger.addState(spatialState,observation);
			
		    QVector<double> max(numDestinations,0.0);
		    int index=0;
			for(unsigned int j=0; j < newB.size();j++)
			{
				index = (int)(j/numStates);
				if( newB(j) && (newB(j) > max[index]) )
				{
					dataLock.lockForWrite();
						destBelief[index] = max[index] = newB(j);
					dataLock.unlock();
					printf("\n Belief is now Updated to Dest:%d State %d with %f",index,j,destBelief[index]);
				}
			}

		  	action = em->chooseAction();			
			followActionToNextState();
			if (em->getStateIsTerminal())
		    {
				printf("  [belief is terminal, ending trial]\n");
				activityLogger.startNewTask();
				break;
		    }
		    printf("\n New Chosen Action is:%d",action);
		}
	}	
}

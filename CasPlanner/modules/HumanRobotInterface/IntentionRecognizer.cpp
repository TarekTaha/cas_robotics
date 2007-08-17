#include "IntentionRecognizer.h"

IntentionRecognizer::IntentionRecognizer():
runRecognition(false),
beliefInitialized(false),
numDestinations(6),
numStates(49),
destBelief(numDestinations,0)
{
	InitializePOMDP();
}

IntentionRecognizer::IntentionRecognizer(PlayGround * playG, RobotManager *rManager):
runRecognition(false),
beliefInitialized(false),
numDestinations(6),
numStates(49),
destBelief(numDestinations,0),
playGround(playG),
robotManager(rManager)
{
	InitializePOMDP();	
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

////  MDPExec* e = em;
//  printf("Number of Actions is:%d\n",em->mdp->getNumActions());
////  belief_vector b(((Pomdp*)em->mdp)->getBeliefSize());
//  belief_vector b;
//  dvector initialBeliefD;
//  initialBeliefD.resize(((Pomdp*)em->mdp)->getBeliefSize());  
//  //belief_vector b = ((Pomdp*)em->mdp)->getInitialBelief();
////  initialBeliefD(5)=1;
////  copy(b, initialBeliefD);
////  for(int i=0; i < b.size();i++)
////  {
////  	if(b(i))
////  	{
////	  	printf("\nBelief%d=%f",i,b(i));
////	  	fflush(stdout);
////  	}
////  }
////  em->setBelief(b);
//
////  printf("  reset to initial belief\n");
//    em->setToInitialState();
////  belief_vector newB = em->currentState;
////  for(int i=0; i < newB.size();i++)
////  {
////  	if(newB(i))
////  	{
////	  	printf("\nBelief%d=%f",i,newB(i));
////	  	fflush(stdout);
////  	}
////  } 
//  int obs[]={2,2,2,2,2,2,2,2,2,2,2,2,2,0,4};
//  int NUM_TRIALS = 15 ;
//  for (int i=0; i < NUM_TRIALS; i++) 
//  {
//	    printf("  step %d\n", i);
//	    int a = em->chooseAction();
//	    printf("    chose action %d\n", a);
//	    int o = obs[i];//em->getRandomOutcome(a);
//	    em->advanceToNextState(a,o);
//	    printf("    updated belief\n");
//	    belief_vector newB = em->currentState;
//	    double max = 0;
//	    int index=0;
//		for(unsigned int j=0; j < newB.size();j++)
//		{
//			if(newB(j)&& newB(j)>max)
//			{
//				max=newB(j);
//				index=j;
//			}
//		}
//		printf("\nNew Belief: %d=%f",index,max);
//		fflush(stdout);
//		if (em->getStateIsTerminal())
//	    {
//			printf("  [belief is terminal, ending trial]\n");
//			break;
//	    } 
//    }		
}

void IntentionRecognizer::run()
{
	Pose location,goToState(-1,-1,-1),oldGoToState(0,0,0);
	int  o=4,a=4,spatialState=-1,oldSpatialState=-1, sp;
	while(runRecognition)
	{
		msleep(50);
		if(!robotManager->commManager)
		{
			qDebug("\t - Communication Manager Not Initialized");
			continue;
		}
		if(!robotManager->commManager->connected)
		{
			qDebug("\t - Your not Connected to the Robot, Connect First");
			continue;		
		}
		location = robotManager->commManager->getOdomLocation();
		spatialState = playGround->mapManager->mapSkeleton.getCurrentSpatialState(location);
		o = robotManager->commManager->getJoyStickGlobalDir();		
		if(!beliefInitialized)
		{
			/*
			 *  When we first connect the readings might not be correct
			 *  so we need to pose to let the commManager properly initialize then 
			 *  request the odom correctly.
			 */
			sleep(1);
			location = robotManager->commManager->getOdomLocation();
			oldSpatialState = spatialState = playGround->mapManager->mapSkeleton.getCurrentSpatialState(location);			
		  	belief_vector b;
		  	dvector initialBeliefD;
		  	initialBeliefD.resize(((Pomdp*)em->mdp)->getBeliefSize());  
		  	for(int i=0;i < numDestinations;i++)
		  	{
		  		initialBeliefD((49*i) + spatialState) = 1/float(numDestinations);
		  	}
		  	copy(b, initialBeliefD);
		  	em->setBelief(b);		
		  	beliefInitialized = true;
		  	a = em->chooseAction();
		}
//		printf("\nJoyStick Global Direction=%d Current Location X=%f, Y=%f CurrentState=%d",robotManager->commManager->getJoyStickGlobalDir(),location.p.x(),location.p.y(),spatialState);
	    
	    //Chose the appropriate Action
//	    printf("    chose action %d\n", a);
	    
	    // Find where this action will take us to
    	for (sp=0; sp < ((Pomdp*)em->mdp)->getBeliefSize(); sp++) 
	    {
	    	if(((Pomdp*)em->mdp)->T[a](spatialState,sp)>0.9)
	    	{
//				printf("Index=%d %5.3f ",sp,((Pomdp*)em->mdp)->T[a](spatialState,sp));
				nextState = sp;
	    	}
	    }
		goToState.p.setX(playGround->mapManager->mapSkeleton.verticies[nextState].location.x());
		goToState.p.setY(playGround->mapManager->mapSkeleton.verticies[nextState].location.y());
		switch(a)
		{
			case 0:
				goToState.phi = DTOR(90);
				break;
			case 1:
				goToState.phi = DTOR(270);
				break;
			case 2:
				goToState.phi = DTOR(0);
				break;
			case 3:
				goToState.phi = DTOR(180);
				break;
			case 4:
				goToState.phi = DTOR(0);
				break;
			default:
				goToState.phi = DTOR(0);
		}
//		printf("\noldGoto X=%f, Y=%f, GoTo X=%f, Y=%f Action=%d Obs=%f",oldGoToState.p.x(),oldGoToState.p.y(),goToState.p.x(),goToState.p.y());
		if(oldGoToState!=goToState && a!=4 && o!=4)
		{
			robotManager->commManager->vfhGoto(goToState);
			oldGoToState= goToState;
		}
		if(oldSpatialState != spatialState)
		{
		    // Get an Observation
		    o = robotManager->commManager->getJoyStickGlobalDir();
		    if(o==4)
		    	continue;
		    em->advanceToNextState(a,o);
		    printf("    updated belief\n");
		    belief_vector newB = em->currentState;
		    
		    QVector<double> max(numDestinations,0.0);
		    int index=0;
			for(unsigned int j=0; j < newB.size();j++)
			{
				index = (j%numStates);
				if(newB(j)&& (newB(j))>max[index])
				{
					max[index] = newB(j);
				}
			}
//			printf("\nNew Belief: %d=%f",index,max);
			fflush(stdout);
			if (em->getStateIsTerminal())
		    {
				printf("  [belief is terminal, ending trial]\n");
				break;
		    }
		    a = em->chooseAction();
		}
	}	
}

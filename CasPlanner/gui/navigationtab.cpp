#include "navigationtab.h"
#include <QProcess>

NavContainer::~NavContainer()
{
	
}

NavContainer::NavContainer(QWidget *parent,PlayGround *playGround_in)
 : QWidget(parent),
   playGround(playGround_in),   
   currRobot(NULL),
   navControlPanel(this,playGround_in)
{
    QHBoxLayout *vLayout = new QHBoxLayout;

 	mapViewer = new MapViewer(this,playGround,&navControlPanel);		
    vLayout->addWidget(mapViewer,4); 

    vLayout->addWidget(&navControlPanel,1); 
    setLayout(vLayout); 
	if(playGround)
	{
		if(playGround->robotPlatforms.size()>0)
			currRobot = playGround->robotPlatforms[0];
	}
}

NavControlPanel::NavControlPanel(NavContainer *container,PlayGround *playG):
	QWidget(container),
	navContainer(container),
	playGround(playG),
	planningGB("Planning"),
	bridgeTest("Bridge Test"),
	connectNodes("Connect Nodes"), 
	regGrid("Generate Reg Grid"), 
	obstPenalty("Obstacle Penalty"),
	expandObst("Expand Obstacles"), 
	showTree("Show Tree"), 
	parametersGB("Planning Parameters"),
	obstExpRadSB(),
	bridgeTestResSB(), 
	bridgeSegLenSB(), 
	regGridResSB(), 
	nodeConRadSB(),
	obstPenRadSB(),
	obstavoidGB("Obstacle Avoidace"),
	noavoidRadBtn("No avoidance /Linear Controller"),
	forceFieldRadBtn("Force Field"),
	configSpaceRadBtn("Local Config Space"),
	vfhRadBtn("VFH"),
	actionGB("Action"),
	pauseBtn("Pause"), 
	pathPlanBtn("Path Plan"),
	generateSpaceBtn("Generate Space"), 
	pathFollowBtn("Path Follow"),
	captureImage("Capture Image"),
	intentionRecognitionBtn("Start IRecognition"),
	robotsGB("Select your Robot"),
	currRobot(NULL),
	robotInitialization(true),
	path(0)
{
	RobotManager *temp= NULL;
	QRadioButton *rob;
	if(playGround)
	{
	    for(int i=0; i < playGround->robotPlatforms.size(); i++)
		{
			temp = playGround->robotPlatforms[i]; 
			rob = new QRadioButton(QString("Robot: ").append(temp->robot->robotName));
			availableRobots.push_back(rob);
		}
		if(playGround->robotPlatforms.size()>0)
			currRobot = playGround->robotPlatforms[0];
	}
	
    QVBoxLayout *hlayout = new QVBoxLayout;
    hlayout->addWidget(&robotsGB,1);
    hlayout->addWidget(&planningGB,1);
    hlayout->addWidget(&parametersGB,1);
    hlayout->addWidget(&obstavoidGB,1);    
    hlayout->addWidget(&actionGB,1); 
    this->setLayout(hlayout);
    QVBoxLayout *showLayout = new QVBoxLayout; 
    showLayout->addWidget(&bridgeTest);
    bridgeTest.setCheckState(Qt::Checked); 
    showLayout->addWidget(&connectNodes);
    connectNodes.setCheckState(Qt::Checked); 
    showLayout->addWidget(&expandObst);
    expandObst.setCheckState(Qt::Checked); 
    showLayout->addWidget(&regGrid);
    regGrid.setCheckState(Qt::Checked); 
    showLayout->addWidget(&obstPenalty);
    obstPenalty.setCheckState(Qt::Checked);
    showLayout->addWidget(&showTree);
    showTree.setCheckState(Qt::Checked);
    planningGB.setLayout(showLayout); 
    
    QGridLayout *parLayout = new QGridLayout;
    parLayout->addWidget(new QLabel("Obstacle Expansion"),0,0);
    parLayout->addWidget(&obstExpRadSB,0,1); 
    parLayout->addWidget(new QLabel("Bridge Test Res"),1,0); 
    parLayout->addWidget(&bridgeTestResSB,1,1);
    parLayout->addWidget(new QLabel("Bridge Length"),2,0); 
    parLayout->addWidget(&bridgeSegLenSB,2,1);
    parLayout->addWidget(new QLabel("Reg Grid Res"),3,0); 
    parLayout->addWidget(&regGridResSB,3,1);
    parLayout->addWidget(new QLabel("Connection Radius"),4,0); 
    parLayout->addWidget(&nodeConRadSB,4,1);
    parLayout->addWidget(new QLabel("Obstacle Penalty"),5,0); 
    parLayout->addWidget(&obstPenRadSB,5,1);
    parametersGB.setLayout(parLayout);
	
	//Loading Default values from config file
    obstExpRadSB.setMinimum(0); 
    obstExpRadSB.setMaximum(1);
	obstExpRadSB.setSingleStep(0.01);

    bridgeTestResSB.setMinimum(0.01); 
    bridgeTestResSB.setMaximum(1);
    bridgeTestResSB.setSingleStep(0.01); 
	    
    bridgeSegLenSB.setMinimum(0.5); 
    bridgeSegLenSB.setMaximum(5);
    bridgeSegLenSB.setSingleStep(0.1);  
	
    regGridResSB.setMinimum(0.02); 
    regGridResSB.setMaximum(5);
    regGridResSB.setSingleStep(0.01);  
	
    nodeConRadSB.setMinimum(0.03); 
    nodeConRadSB.setMaximum(2);
    nodeConRadSB.setSingleStep(0.01);
  
	
    obstPenRadSB.setMinimum(1); 
    obstPenRadSB.setMaximum(5);
    obstPenRadSB.setSingleStep(0.1);


    QVBoxLayout *showL = new QVBoxLayout; 
    showL->addWidget(&noavoidRadBtn);
    showL->addWidget(&forceFieldRadBtn);
    showL->addWidget(&configSpaceRadBtn);
    showL->addWidget(&vfhRadBtn);
    forceFieldRadBtn.setChecked(true);
	updateSelectedAvoidanceAlgo(true);
    obstavoidGB.setLayout(showL); 
    
    QVBoxLayout *robotsL = new QVBoxLayout; 
    for(int i=0;i<availableRobots.size();i++)
    {
    	robotsL->addWidget(availableRobots[i]);
	    connect(availableRobots[i],        SIGNAL(toggled(bool )), this,SLOT(updateSelectedRobot(bool)));    	
    }
       
    QVBoxLayout *actionLayout = new QVBoxLayout; 
    actionLayout->addWidget(&pauseBtn); 
    actionLayout->addWidget(&captureImage);     
    actionLayout->addWidget(&pathPlanBtn);
    actionLayout->addWidget(&generateSpaceBtn); 
    actionLayout->addWidget(&pathFollowBtn); 
    actionLayout->addWidget(&intentionRecognitionBtn);
    actionGB.setLayout(actionLayout); 
    connect(&bridgeTestResSB,  SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&bridgeSegLenSB,   SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&regGridResSB,     SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&nodeConRadSB,     SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&obstPenRadSB,     SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&obstExpRadSB,     SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&vfhRadBtn,        SIGNAL(toggled(bool )), this,SLOT(updateSelectedAvoidanceAlgo(bool)));
    connect(&forceFieldRadBtn, SIGNAL(toggled(bool )), this,SLOT(updateSelectedAvoidanceAlgo(bool)));    
    connect(&configSpaceRadBtn,SIGNAL(toggled(bool )), this,SLOT(updateSelectedAvoidanceAlgo(bool)));    
    connect(&noavoidRadBtn,    SIGNAL(toggled(bool )), this,SLOT(updateSelectedAvoidanceAlgo(bool)));            
	connect(&pathPlanBtn,      SIGNAL(pressed()),this, SLOT(pathPlan()));
	connect(&generateSpaceBtn, SIGNAL(pressed()),this, SLOT(generateSpace()));
	connect(&captureImage,     SIGNAL(pressed()),this, SLOT(save()));	
	connect(&pathFollowBtn,    SIGNAL(pressed()),this, SLOT(pathFollow()));
	connect(&pauseBtn,         SIGNAL(pressed()),this, SLOT(setNavigation()));	
	connect(&intentionRecognitionBtn, SIGNAL(pressed()),this, SLOT(startIntentionRecognition()));

    if(availableRobots.size()>0)
    	availableRobots[0]->setChecked(true);
    robotsGB.setLayout(robotsL); 	
}

void NavControlPanel::startIntentionRecognition()
{
	if(this->currRobot)
	{
		currRobot->startIntentionRecognizer();
	}
	else
	{
		qDebug("No Robot Selected");
	}	
}

void NavControlPanel::pathTraversed()
{
	if(currRobot->navigator->isRunning())
	{
		currRobot->navigator->StopNavigating();
		currRobot->navigator->quit();	
		qDebug("Quitting Thread");
	}
	pathFollowBtn.setText("Path Follow");
	pauseBtn.setText("Pause");
	currRobot->notFollowing = true;
}

void NavControlPanel::pathFollow()
{
	if(currRobot->notFollowing)
	{
		if(currRobot->navigator->isRunning())
		{
			currRobot->navigator->quit();
		}
		if(path)
		{
			currRobot->navigator->setPath(path);
			currRobot->navigator->start();
			pathFollowBtn.setText("Stop");
			currRobot->notFollowing = false;
		}
	}
	else
	{
		if(currRobot->navigator->isRunning())
		{
			currRobot->navigator->StopNavigating();
			currRobot->navigator->quit();	
			qDebug("Quitting Thread");
		}
		pathFollowBtn.setText("Path Follow");
		currRobot->notFollowing = true;
	}
}

void NavControlPanel::setNavigation()
{
	if(currRobot->notPaused)
	{
		currRobot->navigator->setPause(true);
		currRobot->notPaused = false;
		pauseBtn.setText("Continue");
	}
	else
	{
		currRobot->navigator->setPause(false);		
		currRobot->notPaused = true;
		pauseBtn.setText("Pause");		
	}
}

void NavControlPanel::pathPlan()
{
	currRobot->planningManager->setMap(playGround->mapManager->globalMap);
	path = currRobot->planningManager->findPath(METRIC);						
}

void NavControlPanel::generateSpace()
{
	currRobot->planningManager->generateSpace();
}

void NavControlPanel::loadMap()
{

}

void NavControlPanel::updateSelectedObject(double)
{
	if(!robotInitialization)
	{
		if(!currRobot)
			return;
		if(!currRobot->planningManager)
			return;		
		if(currRobot->planningManager->pathPlanner==NULL)
		{
			currRobot->startPlanner();
		}
		currRobot->planningManager->setBridgeTestValue(bridgeSegLenSB.value());
		currRobot->planningManager->setConnNodesValue(nodeConRadSB.value());
		currRobot->planningManager->setRegGridValue(regGridResSB.value());
		currRobot->planningManager->setObstPenValue(obstPenRadSB.value());
		currRobot->planningManager->setExpObstValue(obstExpRadSB.value());
		currRobot->planningManager->setBridgeResValue(bridgeTestResSB.value());
	}
}

void NavControlPanel::updateSelectedRobot(bool)
{
	for(int i=0;i<availableRobots.size();i++)
	{
		if(availableRobots[i]->isChecked())
		{
			currRobot = playGround->robotPlatforms[i];
//			qDebug("Seleted Robot is:%s",qPrintable(currRobot->robot->robotName));	fflush(stdout);
			updateRobotSetting();
			return;
		}
	}
}

void NavControlPanel::updateRobotSetting()
{
	if(!currRobot)
		return;
	if(!currRobot->planningManager)
		return;		
	if(!currRobot->planningManager->pathPlanner)
		currRobot->startPlanner();
	robotInitialization = true;
	obstExpRadSB.setValue(currRobot->planningManager->pathPlanner->obstacle_radius);	
	bridgeTestResSB.setValue(currRobot->planningManager->pathPlanner->bridge_res);
	bridgeSegLenSB.setValue(currRobot->planningManager->pathPlanner->bridge_length);
	regGridResSB.setValue(currRobot->planningManager->pathPlanner->reg_grid);	
	nodeConRadSB.setValue(currRobot->planningManager->pathPlanner->conn_radius);
	obstPenRadSB.setValue(currRobot->planningManager->pathPlanner->obst_dist);
	
    connect(&bridgeTest, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setBridgeTest( int ))); 
    connect(&connectNodes, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setConnNodes( int ))); 
    connect(&regGrid, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setRegGrid( int ))); 
    connect(&obstPenalty, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setObstPen( int ))); 
    connect(&expandObst, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setExpObst( int )));
    connect(&showTree, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setShowTree( int )));

	connect(currRobot->navigator,SIGNAL(pathTraversed()),this,SLOT(pathTraversed()));
	switch(currRobot->navigator->getObstAvoidAlgo())
	{
		case VFH:
		    vfhRadBtn.setChecked(true);		
		    break;
		case FORCE_FIELD:
		    forceFieldRadBtn.setChecked(true);
		    break;
		case CONFIG_SPACE:
		    configSpaceRadBtn.setChecked(true);
		    break;
		case NO_AVOID:
		    noavoidRadBtn.setChecked(true);		
		    break;
		default:
			qDebug("Unkown ALGO");
	}
	
	if(currRobot->notFollowing)
	{
		pathFollowBtn.setText("Path Follow");
	}
	else
	{
		pathFollowBtn.setText("Stop");		
	}
	if(currRobot->notPaused)
	{
		pauseBtn.setText("Pause");
	}
	else
	{
		pauseBtn.setText("Continue");		
	}
	robotInitialization = false;
}

void NavControlPanel::updateSelectedAvoidanceAlgo(bool)
{
	if(!robotInitialization)
	{
		if(currRobot->navigator==NULL)
		{
			currRobot->startNavigator();
		}
		if(vfhRadBtn.isChecked())
		{
			qDebug("VFH");
			currRobot->navigator->setObstAvoidAlgo(VFH);
		}
		else if(forceFieldRadBtn.isChecked())
		{
			qDebug("Force Field");		
			currRobot->navigator->setObstAvoidAlgo(FORCE_FIELD);	
		}
		else if(configSpaceRadBtn.isChecked())
		{
			qDebug("Config Space");		
			currRobot->navigator->setObstAvoidAlgo(CONFIG_SPACE);	
		}
		else if(noavoidRadBtn.isChecked())
		{
			qDebug("NO Avoidace");		
			currRobot->navigator->setObstAvoidAlgo(NO_AVOID);	
		}
	}
}

void NavControlPanel::save()
{
	navContainer->mapViewer->saveImage();
}

void NavControlPanel::setStart(Pose startLoc)
{
	if(this->currRobot)
	{
		currRobot->planningManager->setStart(startLoc);
	}
	else
	{
		qDebug("No Robot is Selected");
	}
}

void NavControlPanel::setEnd(Pose endLoc)
{
	if(this->currRobot)
	{
		currRobot->planningManager->setEnd(endLoc);
	}
	else
	{
		qDebug("No Robot is Selected");
	}	
}

void NavControlPanel::setMap(Map * map)
{
	if(this->currRobot)
	{
		currRobot->planningManager->setMap(map);
	}
	else
	{
		qDebug("No Robot is Selected");
	}		
}

void NavControlPanel::exportHtml()
{
//    QString url=mapManager->exportHtml();
//    QProcess *firefox = new QProcess();
//    QString command = QString("firefox ").append(url);
//    qDebug("Opening %s", qPrintable(command)); 
//    firefox->start(command);
//    firefox->waitForStarted();  
}

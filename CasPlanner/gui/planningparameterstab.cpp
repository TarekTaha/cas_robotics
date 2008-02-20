#include "planningparameterstab.h"

PlanningParametersTab::PlanningParametersTab(QWidget *parent ,PlayGround *playGround):
	QWidget(parent),
	playGround(playGround),
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
//    hlayout->addWidget(&obstavoidGB,1);    
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
//	updateSelectedAvoidanceAlgo(true);
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

PlanningParametersTab::~PlanningParametersTab()
{
}

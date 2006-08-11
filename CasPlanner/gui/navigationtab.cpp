#include "navigationtab.h"
#include <QProcess>

NavContainer::NavContainer(QWidget *parent,RobotManager *rob)
 : QWidget(parent),
   path(0),
   robotManager(rob),   
   navControlPanel(this,rob)
{
    QVBoxLayout *vLayout = new QVBoxLayout; 
    mapPainter = new MapPainter(this,robotManager->mapName);
    vLayout->addWidget(mapPainter,4); 
    vLayout->addWidget(&navControlPanel,1); 
    setLayout(vLayout); 

    connect(&navControlPanel.bridgeTest, SIGNAL(stateChanged(int)),robotManager->planner,SLOT(setBridgeTest( int ))); 
    connect(&navControlPanel.connectNodes, SIGNAL(stateChanged(int)),robotManager->planner,SLOT(setConnNodes( int ))); 
    connect(&navControlPanel.regGrid, SIGNAL(stateChanged(int)),robotManager->planner,SLOT(setRegGrid( int ))); 
    connect(&navControlPanel.obstPenalty, SIGNAL(stateChanged(int)),robotManager->planner,SLOT(setObstPen( int ))); 
    connect(&navControlPanel.expandObst, SIGNAL(stateChanged(int)),robotManager->planner,SLOT(setExpObst( int )));
    connect(&navControlPanel.showTree, SIGNAL(stateChanged(int)),robotManager->planner,SLOT(setShowTree( int )));

	connect(&navControlPanel.pathPlanBtn, SIGNAL(pressed()),this, SLOT(Plan()));
	connect(&navControlPanel.generateSpaceBtn, SIGNAL(pressed()),this, SLOT(GenerateSpace()));
	connect(&navControlPanel.loadMapBtn, SIGNAL(pressed()),this, SLOT(LoadMap()));	
	connect(&navControlPanel.pathFollowBtn, SIGNAL(pressed()),this, SLOT(Follow()));

}

//void NavContainer::rePaint()
//{
//	
//}

void NavContainer::Follow()
{
	if(robotManager->navigator->isRunning())
	{
		robotManager->navigator->quit();
	}
	robotManager->navigator->setPath(path);
	robotManager->navigator->start();
}

NavContainer::~NavContainer()
{
	
}

void NavContainer::Plan()
{
	if(!robotManager->planner->pathPlanner->map)
		robotManager->planner->SetMap(mapPainter->getImage());
	path = robotManager->planner->FindPath(mapPainter->getStart(),mapPainter->getEnd());
	mapPainter->drawPath(robotManager->planner->pathPlanner);
}

void NavContainer::GenerateSpace()
{
	robotManager->planner->SetMap(mapPainter->getImage());
	robotManager->planner->GenerateSpace();
	mapPainter->drawPath(robotManager->planner->pathPlanner);
}

void NavContainer::LoadMap()
{
	robotManager->planner->SetMap(mapPainter->getImage());
	mapPainter->drawPath(robotManager->planner->pathPlanner);
}

NavControlPanel::NavControlPanel(QWidget *parent,RobotManager *rob):
	QWidget(parent),
	robotManager(rob),
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
	noavoidRadBtn("No avoidance"),
	forceFieldRadBtn("Force Field"),
	configSpaceRadBtn("Local Config Space"),
	controlRadBtn("Sensor Based"),
	actionGB("Action"),
	pauseBtn("Pause"), 
	pathPlanBtn("Path Plan"),
	generateSpaceBtn("Generate Space"), 
	pathFollowBtn("Path Follow"),
	loadMapBtn("Load Map"),
	pause(true)
{  
    QHBoxLayout *hlayout = new QHBoxLayout;
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
	obstExpRadSB.setValue(robotManager->planner->pathPlanner->obstacle_radius);

    bridgeTestResSB.setMinimum(0.01); 
    bridgeTestResSB.setMaximum(1);
    bridgeTestResSB.setSingleStep(0.01); 
	bridgeTestResSB.setValue(robotManager->planner->pathPlanner->bridge_res);
	    
    bridgeSegLenSB.setMinimum(0.5); 
    bridgeSegLenSB.setMaximum(5);
    bridgeSegLenSB.setSingleStep(0.1);  
	bridgeSegLenSB.setValue(robotManager->planner->pathPlanner->bridge_length);
	
    regGridResSB.setMinimum(0.02); 
    regGridResSB.setMaximum(5);
    regGridResSB.setSingleStep(0.01);  
	regGridResSB.setValue(robotManager->planner->pathPlanner->reg_grid);
	
    nodeConRadSB.setMinimum(0.03); 
    nodeConRadSB.setMaximum(2);
    nodeConRadSB.setSingleStep(0.01);
    //nodeConRadSB.setDecimals(0);   
	nodeConRadSB.setValue(robotManager->planner->pathPlanner->conn_radius);
	
    obstPenRadSB.setMinimum(1); 
    obstPenRadSB.setMaximum(5);
    obstPenRadSB.setSingleStep(0.1);
   	obstPenRadSB.setValue(robotManager->planner->pathPlanner->obst_dist);
    //obstPenRadSB.setDecimals(0); 

    QVBoxLayout *showL = new QVBoxLayout; 
    showL->addWidget(&noavoidRadBtn);
    showL->addWidget(&forceFieldRadBtn);
    showL->addWidget(&configSpaceRadBtn);
    showL->addWidget(&controlRadBtn);
    configSpaceRadBtn.setChecked(true);
    obstavoidGB.setLayout(showL); 
        
    QVBoxLayout *actionLayout = new QVBoxLayout; 
    actionLayout->addWidget(&pauseBtn); 
    actionLayout->addWidget(&loadMapBtn);     
    actionLayout->addWidget(&pathPlanBtn);
    actionLayout->addWidget(&generateSpaceBtn); 
    actionLayout->addWidget(&pathFollowBtn); 
    actionGB.setLayout(actionLayout); 

    connect(&bridgeTestResSB, SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&bridgeSegLenSB,  SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&regGridResSB,    SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&nodeConRadSB,    SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&obstPenRadSB,    SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&obstExpRadSB,    SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&pauseBtn,      SIGNAL(clicked()), this, SLOT(captureMap())); 
//    connect(&pathPlanBtn, SIGNAL(clicked()), this, SLOT(load())); 
//    connect(&generateSpaceBtn, SIGNAL(clicked()), this, SLOT(save())); 
//    connect(&pathFollowBtn, SIGNAL(clicked()), this, SLOT(exportHtml())); 
}
void NavControlPanel::handleSelection()
{
    qDebug("Item selected ..."); 
    //QTreeWidgetItem *item = selectedObject.currentItem();
    //MapObject *mo = wiToMo[item]; 
    //setActionValues(mo); 
}

void NavControlPanel::setToRoot()
{
    bridgeTestResSB.setValue(0);
    bridgeSegLenSB.setValue(0); 
    regGridResSB.setValue(0); 
    nodeConRadSB.setValue(0); 
    obstPenRadSB.setValue(0);
}

void NavControlPanel::updateSelectedObject(double)
{
	if(robotManager->planner->pathPlanner==NULL)
	{
		robotManager->startPlanner();
	}
	robotManager->planner->setBridgeTestValue(bridgeSegLenSB.value());
	robotManager->planner->setConnNodesValue(nodeConRadSB.value());
	robotManager->planner->setRegGridValue(regGridResSB.value());
	robotManager->planner->setObstPenValue(obstPenRadSB.value());
	robotManager->planner->setExpObstValue(obstExpRadSB.value());
	robotManager->planner->setBridgeResValue(bridgeTestResSB.value());
}

//void navControlPanel::setActionValues(MapObject *mo)
//{
//    int visibility = mo->getVisibility();
//    Vector6DOF pose = mo->getOrigin();
//    obstExpRadSB.setValue(visibility); 
//    bridgeTestResSB.setValue(pose.getX());
//    bridgeSegLenSB.setValue(pose.getY()); 
//    regGridResSB.setValue(pose.getZ()); 
//    nodeConRadSB.setValue(pose.getRollDeg()); 
//    obstPenRadSB.setValue(pose.getPitchDeg());
//    yaSB.setValue(pose.getYawDeg());  
//}

//void navControlPanel::setMapManager( QTMapDataInterface *in_mapManager)
//{
//    //qDebug("set map manager");
//fprintf(stdout, "setMapManager called, parameter=%x\n", in_mapManager); 
//    mapManager = in_mapManager; 
//    connect(mapManager, SIGNAL(patchChanged()), this, SLOT(updateMap())); 
//}

//void NavControlPanel::load()
//{
////fprintf(stdout, "Load button pressed, about to call load on mapmanager %x\n", mapManager); 
////    QString dir = QFileDialog::getExistingDirectory(this, "Load log"); 
////    mapManager->readIn(dir); 
//}

void NavControlPanel::save()
{
//    QString dir = QFileDialog::getSaveFileName(this, "Save log"); 
//    mapManager->writeOut(dir); 
}

void NavControlPanel::captureMap()
{
	if(pause)
	{
		robotManager->navigator->setPause(pause);
		pause = false;
		pauseBtn.setText("Continue");
	}
	else
	{
		robotManager->navigator->setPause(pause);		
		pause = true;
		pauseBtn.setText("Pause");		
	}
//    bool ok;
//    QString comment = QInputDialog::getText(this, "CAS Planner: Save Map", "Enter name for this map:", QLineEdit::Normal,
//    QString::null, &ok);
//    emit propsChanged();
//    sleep(1);   
//    emit propsChanged();
//    sleep(1);
//    if(ok)
//    {
//		//QImage capturedMap = ((NavContainer *) parent())->mapViewer.captureMap();
//		//QImage capturedMap = ((NavContainer *) parent())->mapPainter->captureMap();
//		//mapManager->addMapSnapshot(capturedMap, comment); 
//    }
}

void NavControlPanel::exportHtml()
{
    // This should be in map
//    QString url=mapManager->exportHtml();
//    QProcess *firefox = new QProcess();
//    QString command = QString("firefox ").append(url);
//    qDebug("Opening %s", qPrintable(command)); 
//    firefox->start(command);
//    firefox->waitForStarted();  
}

void NavControlPanel::updateMap()
{
//    QVector<QString> robots = mapManager->getRobotIds();
//    for(int i=0; i < robots.size(); i++){
//	MapObject *robotMO = mapManager->getMORobotMap(robots[i]); 
//	QTreeWidgetItem *robotItem; 
//	if(!moToWi.contains(robotMO)){
//	    robotItem = new QTreeWidgetItem(QStringList(QString("Robot ").append(robots[i])),RobotNodeType);
//	    selectedObject.addTopLevelItem(robotItem); 
//	    moToWi[robotMO] = robotItem; 
//	    wiToMo[robotItem] = robotMO; 
//	    selectedObject.expandItem(robotItem); 
//	}
//	else {
//	    robotItem = moToWi[robotMO];  
//	}
//	QVector<QString> patches = mapManager->getPatchIds(robots[i]);
//	for (int j=0; j < patches.size(); j++){
//	    MapObject *patchMO = mapManager->getMOPatch(robots[i], patches[j]);
//	    QTreeWidgetItem* patchItem; 
//	    if(!moToWi.contains(patchMO)){
//		patchItem = new QTreeWidgetItem(QStringList(QString("Patch ").append(patches[j])), PatchNodeType);
//		robotItem->addChild(patchItem); 
//		moToWi[patchMO] = patchItem; 
//		wiToMo[patchItem] = patchMO; 
//		selectedObject.expandItem(patchItem); 
//	    }
//	    else {
//		patchItem = moToWi[patchMO]; 
//	    }
//	    QVector<Snap *> snaps = mapManager->getMOSnaps(robots[i], patches[j]); 
//	    for(int k=0; k < snaps.size(); k++){
//		MapObject *snapMO = snaps[k]; 
//		if(!moToWi.contains(snapMO)){
//		    QTreeWidgetItem *snapItem;
//		    orca::SnapPtr oSnap = snaps[k]->getSnap();
//		    if(oSnap->type == orca::SNAPVICTIM){
//			snapItem = new QTreeWidgetItem(QStringList(QString("Victim: ").append(oSnap->desc.c_str())), SnapNodeType);
//		    }
//		    else {
//			snapItem = new QTreeWidgetItem(QStringList(QString("Landmark: ").append(oSnap->desc.c_str())));	
//		    }
//		    patchItem->addChild(snapItem); 
//		    moToWi[snapMO] = snapItem; 
//		    wiToMo[snapItem] = snapMO; 
//		}
//	    }
//	    QVector<Pointcloud *> pointclouds = mapManager->getMOPointclouds(robots[i], patches[j]); 
//	    for(int k=0; k < pointclouds.size(); k++){
//		MapObject *pointcloudMO = pointclouds[k];
//		if(!moToWi.contains(pointcloudMO)){
//		    QTreeWidgetItem *pointcloudItem = new QTreeWidgetItem(QStringList(QString("Pointcloud ").append(k)));
//		    patchItem->addChild(pointcloudItem); 
//		    moToWi[pointcloudMO] = pointcloudItem; 
//		    wiToMo[pointcloudItem] = pointcloudMO; 
//		    patchItem->addChild(pointcloudItem); 
//		}
//
//	    }
//	}
//    }
    
}
    

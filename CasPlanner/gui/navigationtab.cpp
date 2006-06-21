#include "navigationtab.h"
#include <QProcess>

NavContainer::NavContainer(QWidget *parent)
 : QWidget(parent),
   mapViewer(this), 
   navControlPanel(this)
{
    QVBoxLayout *vLayout = new QVBoxLayout; 
    vLayout->addWidget(&mapViewer,4); 
    vLayout->addWidget(&navControlPanel,1); 
    setLayout(vLayout); 
    connect(&navControlPanel.showGrids, SIGNAL(stateChanged(int)),&mapViewer,SLOT(setShowGrids( int ))); 
    connect(&navControlPanel.showOGs, SIGNAL(stateChanged(int)),&mapViewer,SLOT(setShowOGs( int ))); 
    connect(&navControlPanel.showRobots, SIGNAL(stateChanged(int)),&mapViewer,SLOT(setShowRobots( int ))); 
    connect(&navControlPanel.showLabels, SIGNAL(stateChanged(int)),&mapViewer,SLOT(setShowSnaps( int ))); 
    connect(&navControlPanel.showPointclouds, SIGNAL(stateChanged(int)),&mapViewer,SLOT(setShowPointclouds( int )));
    connect(&navControlPanel.showPatchBorders, SIGNAL(stateChanged(int)),&mapViewer,SLOT(setShowPatchBorders( int )));
    connect(&navControlPanel, SIGNAL(propsChanged()), &mapViewer, SLOT(update()));  
    connect(&mapViewer, SIGNAL(moveMOLeft()), &(navControlPanel.xSB), SLOT(stepDown())); 
    connect(&mapViewer, SIGNAL(moveMORight()), &(navControlPanel.xSB), SLOT(stepUp())); 
    connect(&mapViewer, SIGNAL(moveMOUp()), &(navControlPanel.ySB), SLOT(stepUp())); 
    connect(&mapViewer, SIGNAL(moveMODown()), &(navControlPanel.ySB), SLOT(stepDown())); 
    connect(&mapViewer, SIGNAL(yawMOPos()), &(navControlPanel.yaSB), SLOT(stepUp())); 
    connect(&mapViewer, SIGNAL(yawMONeg()), &(navControlPanel.yaSB), SLOT(stepDown()));
}

//void MapEdit::setMapManager( QTMapDataInterface *mapManager){
//    mapViewer.setMapManager(mapManager); 
//    navControlPanel.setMapManager(mapManager);
//}

NavContainer::~NavContainer()
{
}

NavControlPanel::NavControlPanel( QWidget *parent):
	QWidget(parent),
	showHideGB("Show/Hide"),
	showGrids("Grids"),
	showOGs("Occ grids"), 
	showRobots("Robots"), 
	showLabels("Victims and LMs"),
	showPointclouds("Pointclouds"), 
	showPatchBorders("Patch Borders"), 
	selectedObject(), 
	transformGB("Properties"),
	visSB(),
	xSB(), 
	ySB(), 
	zSB(), 
	rSB(),
	pSB(),
	yaSB(),
	setToRootBtn("Set to Root"), 
	actionGB("Action"),
	captureMapBtn("Capture Map ..."), 
	loadBtn("Load ..."),
	saveAsBtn("Save as ..."), 
	exportBtn("Export HTML")
{
    selectedObject.setColumnCount(1); 
    selectedObject.setHeaderLabels(QStringList("Select")); 
    QHBoxLayout *hlayout = new QHBoxLayout;
    hlayout->addWidget(&showHideGB,1);
    hlayout->addWidget(&selectedObject,1);
    hlayout->addWidget(&transformGB,1);
    hlayout->addWidget(&actionGB,1); 
    this->setLayout(hlayout);
    QVBoxLayout *showLayout = new QVBoxLayout; 
    showLayout->addWidget(&showGrids);
    showGrids.setCheckState(Qt::Checked); 
    showLayout->addWidget(&showOGs);
    showOGs.setCheckState(Qt::Checked); 
    showLayout->addWidget(&showPointclouds);
    showPointclouds.setCheckState(Qt::Checked); 
    showLayout->addWidget(&showRobots);
    showRobots.setCheckState(Qt::Checked); 
    showLayout->addWidget(&showLabels);
    showLabels.setCheckState(Qt::Checked);
    showLayout->addWidget(&showPatchBorders);
    showPatchBorders.setCheckState(Qt::Checked);
    showHideGB.setLayout(showLayout); 
    QGridLayout *xfrmLayout = new QGridLayout;
    xfrmLayout->addWidget(new QLabel("Visible"),0,0);
    xfrmLayout->addWidget(&visSB,0,1); 
    xfrmLayout->addWidget(new QLabel("x"),1,0); 
    xfrmLayout->addWidget(&xSB,1,1);
    xfrmLayout->addWidget(new QLabel("y"),2,0); 
    xfrmLayout->addWidget(&ySB,2,1);
    xfrmLayout->addWidget(new QLabel("z"),3,0); 
    xfrmLayout->addWidget(&zSB,3,1);
    xfrmLayout->addWidget(new QLabel("roll"),4,0); 
    xfrmLayout->addWidget(&rSB,4,1);
    xfrmLayout->addWidget(new QLabel("pitch"),5,0); 
    xfrmLayout->addWidget(&pSB,5,1);
    xfrmLayout->addWidget(new QLabel("yaw"),6,0); 
    xfrmLayout->addWidget(&yaSB,6,1); 
    xfrmLayout->addWidget(&setToRootBtn,7,0,1,2);  
    transformGB.setLayout(xfrmLayout);
    visSB.setMinimum(0); 
    visSB.setMaximum(Qt::Checked);
    xSB.setMinimum(-25.0); 
    xSB.setMaximum(25.0);
    xSB.setSingleStep(0.1); 
    ySB.setMinimum(-25.0); 
    ySB.setMaximum(25.0);
    ySB.setSingleStep(0.1);  
    zSB.setMinimum(-25.0); 
    zSB.setMaximum(25.0);
    zSB.setSingleStep(0.1);  
    rSB.setMinimum(-90); 
    rSB.setMaximum(90);
    rSB.setSingleStep(1);
    rSB.setDecimals(0);   
    pSB.setMinimum(-90); 
    pSB.setMaximum(90);
    pSB.setSingleStep(1);
    pSB.setDecimals(0); 
    yaSB.setMinimum(-180); 
    yaSB.setMaximum(180);
    yaSB.setSingleStep(1);
    yaSB.setDecimals(0); 
    QVBoxLayout *actionLayout = new QVBoxLayout; 
    actionLayout->addWidget(&captureMapBtn); 
    actionLayout->addWidget(&loadBtn);
    actionLayout->addWidget(&saveAsBtn); 
    actionLayout->addWidget(&exportBtn); 
    actionGB.setLayout(actionLayout); 
    connect(&selectedObject, SIGNAL(itemSelectionChanged()), this, SLOT(handleSelection()));
    connect(&xSB, SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&ySB, SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&zSB, SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&rSB, SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&pSB, SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&yaSB, SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&visSB, SIGNAL(valueChanged(int)), this, SLOT(updateSelectedObject(int)));
    connect(&captureMapBtn, SIGNAL(clicked()), this, SLOT(captureMap())); 
    connect(&loadBtn, SIGNAL(clicked()), this, SLOT(load())); 
    connect(&saveAsBtn, SIGNAL(clicked()), this, SLOT(save())); 
    connect(&exportBtn, SIGNAL(clicked()), this, SLOT(exportHtml())); 
    connect(&setToRootBtn, SIGNAL(clicked()), this, SLOT(setToRoot())); 
}

void NavControlPanel::handleSelection(){
    qDebug("Item selected ..."); 
    QTreeWidgetItem *item = selectedObject.currentItem();
    //MapObject *mo = wiToMo[item]; 
    //setActionValues(mo); 
}

void NavControlPanel::setToRoot(){
    xSB.setValue(0);
    ySB.setValue(0); 
    zSB.setValue(0); 
    rSB.setValue(0); 
    pSB.setValue(0);
    yaSB.setValue(0);  
}

void NavControlPanel::updateSelectedObject(double)
{
//    qDebug("Updating selected item"); 
//    QTreeWidgetItem *item = selectedObject.currentItem();
//    if(item != NULL && wiToMo.contains(item))
//    {
//	MapObject *mo = wiToMo[item]; 
//	mo->setVisibility(visSB.value()); 
//	Vector6DOF newPose;
//	newPose.setX(xSB.value());
//	newPose.setY(ySB.value());
//	newPose.setZ(zSB.value());
//	newPose.setPitchDeg(pSB.value());
//	newPose.setRollDeg(rSB.value());
//	newPose.setYawDeg(yaSB.value());
//	mo->setOrigin(newPose); 
//	emit propsChanged();
//    }
//  else 
//	{
//		qDebug("Item unselected!!!"); 
//  }
}

void NavControlPanel::updateSelectedObject(int value)
{
    updateSelectedObject((double) value); 
}

//void navControlPanel::setActionValues(MapObject *mo)
//{
//    int visibility = mo->getVisibility();
//    Vector6DOF pose = mo->getOrigin();
//    visSB.setValue(visibility); 
//    xSB.setValue(pose.getX());
//    ySB.setValue(pose.getY()); 
//    zSB.setValue(pose.getZ()); 
//    rSB.setValue(pose.getRollDeg()); 
//    pSB.setValue(pose.getPitchDeg());
//    yaSB.setValue(pose.getYawDeg());  
//}

//void navControlPanel::setMapManager( QTMapDataInterface *in_mapManager)
//{
//    //qDebug("set map manager");
//fprintf(stdout, "setMapManager called, parameter=%x\n", in_mapManager); 
//    mapManager = in_mapManager; 
//    connect(mapManager, SIGNAL(patchChanged()), this, SLOT(updateMap())); 
//}

void NavControlPanel::load()
{
//fprintf(stdout, "Load button pressed, about to call load on mapmanager %x\n", mapManager); 
//    QString dir = QFileDialog::getExistingDirectory(this, "Load log"); 
//    mapManager->readIn(dir); 
}

void NavControlPanel::save()
{
//    QString dir = QFileDialog::getSaveFileName(this, "Save log"); 
//    mapManager->writeOut(dir); 
}

void NavControlPanel::captureMap()
{
    bool ok;
    QString comment = QInputDialog::getText(this, "RescueGUI: Capture Comment", "Enter a comment for this map:", QLineEdit::Normal,
	    QString::null, &ok);
    emit propsChanged();
    sleep(1);   
    emit propsChanged();
    sleep(1);
    if(ok){
	QImage capturedMap = ((NavContainer *) parent())->mapViewer.captureMap();
	//mapManager->addMapSnapshot(capturedMap, comment); 
    }
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
    

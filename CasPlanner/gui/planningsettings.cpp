#include "planningsettings.h"
#include "ui_planningsettings.h"

#include "playground.h"
#include "settings.h"
#include "planningmanager.h"
#include "navigator.h"
#include "mapviewer.h"

PlanningSettings::PlanningSettings(QWidget *parent, PlayGround *playG) :
        QWidget(parent),
        playGround(playG),
        currRobot(NULL),
        ui(new Ui::PlanningSettings)
{
    ui->setupUi(this);
    initialiseGUI();
    connect(ui->bridgeTestResSB,  SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(ui->bridgeSegLenSB,   SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(ui->regGridResSB,     SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(ui->nodeConRadSB,     SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(ui->obstPenRadSB,     SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(ui->obstExpRadSB,     SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(ui->vfhRadBtn,        SIGNAL(toggled(bool )), this,SLOT(updateSelectedAvoidanceAlgo(bool)));
    connect(ui->noavoidRadBtn,    SIGNAL(toggled(bool )), this,SLOT(updateSelectedAvoidanceAlgo(bool)));

    connect(ui->bridgeTest,         SIGNAL(stateChanged(int)),this,SLOT(updateCheckboxStates(int)));
    connect(ui->connectNodes,       SIGNAL(stateChanged(int)),this,SLOT(updateCheckboxStates(int)));
    connect(ui->regGrid,            SIGNAL(stateChanged(int)),this,SLOT(updateCheckboxStates(int)));
    connect(ui->obstPenalty,        SIGNAL(stateChanged(int)),this,SLOT(updateCheckboxStates(int)));
    connect(ui->expandObst,         SIGNAL(stateChanged(int)),this,SLOT(updateCheckboxStates(int)));

    connect(ui->showSearchTree,     SIGNAL(stateChanged(int)),this,SLOT(updateDisplaySettings(int)));
    connect(ui->showSearchSpaceTree,SIGNAL(stateChanged(int)),this,SLOT(updateDisplaySettings(int)));
    connect(ui->showPaths,          SIGNAL(stateChanged(int)),this,SLOT(updateDisplaySettings(int)));
    connect(ui->showRobotTrail,     SIGNAL(stateChanged(int)),this,SLOT(updateDisplaySettings(int)));
    connect(ui->showSearchSpaceSamples,SIGNAL(stateChanged(int)),this,SLOT(updateDisplaySettings(int)));
}

PlanningSettings::~PlanningSettings()
{
    delete ui;
}

void PlanningSettings::initialiseGUI()
{
    ui->obstExpRadSB->setValue(CasPlanner::settings().obstacleExpandR());
    ui->bridgeTestResSB->setValue(CasPlanner::settings().bridgeTestRes());
    ui->bridgeSegLenSB->setValue(CasPlanner::settings().bridgeLength());
    ui->regGridResSB->setValue(CasPlanner::settings().regGridRes());
    ui->nodeConRadSB->setValue(CasPlanner::settings().nodeConnectionR());
    ui->obstPenRadSB->setValue(CasPlanner::settings().obstaclePenR());

    ui->bridgeTest->setChecked(CasPlanner::settings().isBridgeTestEnabled());
    ui->connectNodes->setChecked(CasPlanner::settings().isConnectNodesEnabled());
    ui->expandObst->setChecked(CasPlanner::settings().isExpandObstEnabled());
    ui->regGrid->setChecked(CasPlanner::settings().isRegGridEnabled());
    ui->obstPenalty->setChecked(CasPlanner::settings().isObstaclePenEnabled());

    ui->showPaths->setChecked(CasPlanner::settings().isShowPathsEnabled());
    ui->showSearchSpaceSamples->setChecked(CasPlanner::settings().isShowSearchSpaceSamplesEnabled());
    ui->showSearchSpaceTree->setChecked(CasPlanner::settings().isShowSearchSpaceTreeEnabled());
    ui->showSearchTree->setChecked(CasPlanner::settings().isShowSearchTreeEnabled());
    ui->showRobotTrail->setChecked(CasPlanner::settings().isShowRobotTrailEnabled());

    switch(CasPlanner::settings().obstacleAvoidAlgo())
    {
    case VFH:
        ui->vfhRadBtn->setChecked(true);
        break;
    case NO_AVOID:
        ui->noavoidRadBtn->setChecked(true);
        break;
    default:
        qDebug()<<"Unkown ALGO";
    }
}

void PlanningSettings::saveSettings()
{
    CasPlanner::settings().setObstacleExpandR(ui->obstExpRadSB->value());
    CasPlanner::settings().setObstaclePenR(ui->obstPenRadSB->value());
    CasPlanner::settings().setBridgeTestRes(ui->bridgeTestResSB->value());
    CasPlanner::settings().setBridgeLength(ui->bridgeSegLenSB->value());
    CasPlanner::settings().setRegGridRes(ui->regGridResSB->value());
    CasPlanner::settings().setNodeConnectionR(ui->nodeConRadSB->value());

    CasPlanner::settings().setBridgeTest(ui->bridgeTest->isChecked());
    CasPlanner::settings().setConnectNodes(ui->connectNodes->isChecked());
    CasPlanner::settings().setExpandObst(ui->expandObst->isChecked());
    CasPlanner::settings().setRegGrid(ui->regGrid->isChecked());
    CasPlanner::settings().setObstaclePen(ui->obstPenalty->isChecked());

    CasPlanner::settings().setShowPaths(ui->showPaths->isChecked());
    CasPlanner::settings().setShowSearchSpaceSamples(ui->showSearchSpaceSamples->isChecked());
    CasPlanner::settings().setShowSearchSpaceTree(ui->showSearchSpaceTree->isChecked());
    CasPlanner::settings().setShowSearchTree(ui->showSearchTree->isChecked());
    CasPlanner::settings().setShowRobotTrail(ui->showRobotTrail->isChecked());

    if(ui->vfhRadBtn->isChecked())
    {
        CasPlanner::settings().setObstacleAvoidAlgo(VFH);
    }
    else if(ui->noavoidRadBtn->isChecked())
    {
        CasPlanner::settings().setObstacleAvoidAlgo(NO_AVOID);
    }
}

void PlanningSettings::updateDisplaySettings(int)
{
    if(playGround->mapViewer)
    {
        playGround->mapViewer->setShowSearchTree(ui->showSearchTree->isChecked());
        playGround->mapViewer->setShowSearchSpaceSamples(ui->showSearchSpaceSamples->isChecked());
        playGround->mapViewer->setShowSearchSpaceTree(ui->showSearchSpaceTree->isChecked());
        playGround->mapViewer->setShowPath(ui->showPaths->isChecked());
        playGround->mapViewer->setShowRobotTrail(ui->showRobotTrail->isChecked());
    }
    saveSettings();
}

void PlanningSettings::updateCheckboxStates(int)
{
    for (int i=0;i<playGround->robotPlatforms.size();i++)
    {
        currRobot = playGround->robotPlatforms[i];
        if(!currRobot)
            break;
        if(!currRobot->planningManager)
            break;
        currRobot->planningManager->setBridgeTest(ui->bridgeTest->isChecked());
        currRobot->planningManager->setConnNodes(ui->connectNodes->isChecked());
        currRobot->planningManager->setRegGrid(ui->regGrid->isChecked());
        currRobot->planningManager->setObstPen(ui->obstPenalty->isChecked());
        currRobot->planningManager->setExpObst(ui->expandObst->isChecked());
    }
    saveSettings();
}

void PlanningSettings::updateSelectedObject(double)
{
    for (int i=0;i<playGround->robotPlatforms.size();i++)
    {
        currRobot = playGround->robotPlatforms[i];
        if(!currRobot)
            break;
        if(!currRobot->planningManager)
            break;
        if(currRobot->planningManager->pathPlanner==NULL)
        {
            currRobot->startPlanner();
        }
        currRobot->planningManager->setBridgeTestValue(ui->bridgeSegLenSB->value());
        currRobot->planningManager->setConnNodesValue(ui->nodeConRadSB->value());
        currRobot->planningManager->setRegGridValue(ui->regGridResSB->value());
        currRobot->planningManager->setObstPenValue(ui->obstPenRadSB->value());
        currRobot->planningManager->setExpObstValue(ui->obstExpRadSB->value());
        currRobot->planningManager->setBridgeResValue(ui->bridgeTestResSB->value());
    }
    saveSettings();
}

void PlanningSettings::updateSelectedAvoidanceAlgo(bool)
{
    for (int i=0;i<playGround->robotPlatforms.size();i++)
    {
        currRobot = playGround->robotPlatforms[i];
        if(currRobot->navigator==NULL)
        {
            currRobot->startNavigator();
        }
        if(ui->vfhRadBtn->isChecked())
        {
            qDebug()<<"VFH";
            currRobot->navigator->setObstAvoidAlgo(VFH);
        }
        else if(ui->noavoidRadBtn->isChecked())
        {
            qDebug()<<"NO Avoidace";
            currRobot->navigator->setObstAvoidAlgo(NO_AVOID);
        }
    }
    saveSettings();
}

#include "planningsettings.h"
#include "ui_planningsettings.h"

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
    if(playGround->robotPlatforms.size()>0)
    {
        // Populate the GUI with the settings of the first ROBOT
        currRobot = playGround->robotPlatforms[0];
        if(!currRobot)
            return;
        if(!currRobot->planningManager)
            return;
        if(!currRobot->planningManager->pathPlanner)
            currRobot->startPlanner();
        ui->obstExpRadSB->setValue(currRobot->planningManager->pathPlanner->obstacle_radius);
        ui->bridgeTestResSB->setValue(currRobot->planningManager->pathPlanner->bridge_res);
        ui->bridgeSegLenSB->setValue(currRobot->planningManager->pathPlanner->bridge_length);
        ui->regGridResSB->setValue(currRobot->planningManager->pathPlanner->regGridDist);
        ui->nodeConRadSB->setValue(currRobot->planningManager->pathPlanner->reg_grid_conn_rad);
        ui->obstPenRadSB->setValue(currRobot->planningManager->pathPlanner->obst_dist);
        switch(currRobot->navigator->getObstAvoidAlgo())
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
}

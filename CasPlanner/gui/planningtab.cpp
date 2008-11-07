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
#include "planningtab.h"

PlanningTab::~PlanningTab()
{
}

PlanningTab::PlanningTab(QWidget *parent, PlayGround *playG):
    QWidget(parent),
    playGround(playG),
    planningGB("Planning"),
    bridgeTest("Bridge Test"),
    connectNodes("Connect Nodes"),
    regGrid("Generate Reg Grid"),
    obstPenalty("Obstacle Penalty"),
    expandObst("Expand Obstacles"),
    showSearchTree("Show Search  Tree"),
    showSearchSpaceTree("Show Search Space"),
    showPaths("Show Paths"),
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
    robotsGB("Select your Robot"),
    currRobot(NULL),
    robotInitialization(true)
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
    showLayout->addWidget(&showSearchTree);
    showSearchTree.setCheckState(Qt::Unchecked);
    showLayout->addWidget(&showSearchSpaceTree);
    showSearchSpaceTree.setCheckState(Qt::Unchecked);
    showLayout->addWidget(&showPaths);
    showPaths.setCheckState(Qt::Checked);
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

    if(availableRobots.size()>0)
        availableRobots[0]->setChecked(true);
    robotsGB.setLayout(robotsL);
}

void PlanningTab::updateSelectedObject(double)
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

void PlanningTab::updateRobotSetting()
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
    regGridResSB.setValue(currRobot->planningManager->pathPlanner->regGridDist);
    nodeConRadSB.setValue(currRobot->planningManager->pathPlanner->reg_grid_conn_rad);
    obstPenRadSB.setValue(currRobot->planningManager->pathPlanner->obst_dist);

    connect(&bridgeTest, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setBridgeTest( int )));
    connect(&connectNodes, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setConnNodes( int )));
    connect(&regGrid, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setRegGrid( int )));
    connect(&obstPenalty, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setObstPen( int )));
    connect(&expandObst, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setExpObst( int )));
    connect(&showSearchSpaceTree, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setShowSearchSpaceTree( int )));
    connect(&showSearchTree, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setShowSearchTree( int )));
    connect(&showPaths, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setShowPaths( int )));

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

    robotInitialization = false;
}

void PlanningTab::updateSelectedAvoidanceAlgo(bool)
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

void PlanningTab::updateSelectedRobot(bool)
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


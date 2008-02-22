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
    hlayout->addWidget(&actionGB,1); 

    this->setLayout(hlayout);
	
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
	if(!currRobot->commManager)
	{
		qDebug("\t NavTab: Communication Manager Not Initialized");
		return;
	}
	if(!currRobot->commManager->connected)
	{
		QMessageBox msgBox(QMessageBox::Warning,QString("Warning"),QString("Your not Connected to the Robot, do you want me to connect?"),QMessageBox::Ok|QMessageBox::Cancel,this,Qt::Dialog);
		msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
		switch (msgBox.exec()) 
		{
		case QMessageBox::Yes:
			currRobot->startComms();
		    break;
		case QMessageBox::No:
		    return;
		   break;
		default:
		    return;
		    break;
		}			
	}
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

void NavControlPanel::updateSelectedRobot(bool)
{
	for(int i=0;i<availableRobots.size();i++)
	{
		if(availableRobots[i]->isChecked())
		{
			currRobot = playGround->robotPlatforms[i];
//			qDebug("Seleted Robot is:%s",qPrintable(currRobot->robot->robotName));	fflush(stdout);
//			if(!currRobot)
//				return;
//			if(!currRobot->planningManager)
//				return;		
//			if(!currRobot->planningManager->pathPlanner)
//				currRobot->startPlanner();
			return;
		}
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

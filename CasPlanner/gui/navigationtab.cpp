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
			playGround->activeRobot = playGround->robotPlatforms[0];	
	}
}

NavControlPanel::NavControlPanel(NavContainer *container,PlayGround *playG):
	QWidget(container),
	navContainer(container),
	playGround(playG),
	actionGB("Action"),
	pauseBtn("Pause"), 
	pathPlanBtn("Path Plan"),
//	generateSpaceBtn("Generate Space"), 
	resetBeliefBtn("Reset Belief"),
	pathFollowBtn("Path Follow"),
	captureImage("Capture Image"),
	intentionRecognitionBtn("Start IRecognition"),
	robotsGB("Select your Robot"),
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
			playGround->activeRobot = playGround->robotPlatforms[0];
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
//    actionLayout->addWidget(&generateSpaceBtn); 
    actionLayout->addWidget(&pathFollowBtn); 
    actionLayout->addWidget(&intentionRecognitionBtn);
    actionLayout->addWidget(&resetBeliefBtn);
    actionGB.setLayout(actionLayout); 
	connect(&pathPlanBtn,      SIGNAL(pressed()),this, SLOT(pathPlan()));
//	connect(&generateSpaceBtn, SIGNAL(pressed()),this, SLOT(generateSpace()));
	connect(&captureImage,     SIGNAL(pressed()),this, SLOT(save()));	
	connect(&pathFollowBtn,    SIGNAL(pressed()),this, SLOT(pathFollow()));
	connect(&pauseBtn,         SIGNAL(pressed()),this, SLOT(setNavigation()));	
	connect(&intentionRecognitionBtn, SIGNAL(pressed()),this, SLOT(startIntentionRecognition()));
	connect(&resetBeliefBtn, 	   SIGNAL(pressed()),this, SLOT(resetDestinationBelief()));	
    if(availableRobots.size()>0)
    	availableRobots[0]->setChecked(true);
    robotsGB.setLayout(robotsL); 	
}

void NavControlPanel::resetDestinationBelief()
{
	if(this->playGround->activeRobot)
	{
		if(!playGround->activeRobot->intentionRecognizer)
		{
			QMessageBox msgBox(QMessageBox::Warning,QString("Warning"),QString("Intention Recognition not started, you want to start it now?"),QMessageBox::Ok|QMessageBox::Cancel,this,Qt::Dialog);
			msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
			switch (msgBox.exec()) 
			{
			case QMessageBox::Yes:
				startIntentionRecognition();
			    break;
			case QMessageBox::No:
			    return;
			   break;
			default:
			    return;
			    break;
			}	
		}
		playGround->activeRobot->intentionRecognizer->resetBelief();
	}
	else
	{
		qDebug("Select a Robot and Start the intention Recognizer First !!!");
	}	
}

void NavControlPanel::startIntentionRecognition()
{
	if(this->playGround->activeRobot)
	{
		if(!playGround->activeRobot->commManager->isConnected())
		{
			QMessageBox msgBox(QMessageBox::Warning,QString("Warning"),QString("Your not Connected to the Robot, do you want me to connect?"),QMessageBox::Ok|QMessageBox::Cancel,this,Qt::Dialog);
			msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
			switch (msgBox.exec()) 
			{
			case QMessageBox::Yes:
				playGround->activeRobot->startComms();
			    break;
			case QMessageBox::No:
			    return;
			   break;
			default:
			    return;
			    break;
			}			
		}		
		playGround->activeRobot->startIntentionRecognizer();
	}
	else
	{
		qDebug("No Robot Selected");
	}	
}

void NavControlPanel::pathTraversed()
{
	if(playGround->activeRobot->navigator->isRunning())
	{
		playGround->activeRobot->navigator->StopNavigating();
		playGround->activeRobot->navigator->quit();	
		qDebug("Quitting Thread");
	}
	pathFollowBtn.setText("Path Follow");
	pauseBtn.setText("Pause");
	playGround->activeRobot->notFollowing = true;
}

void NavControlPanel::pathFollow()
{
	if(!playGround->activeRobot->commManager)
	{
		qDebug("\t NavTab: Communication Manager Not Initialized");
		return;
	}
	if(!playGround->activeRobot->commManager->isConnected())
	{
		QMessageBox msgBox(QMessageBox::Warning,QString("Warning"),QString("Your not Connected to the Robot, do you want me to connect?"),QMessageBox::Ok|QMessageBox::Cancel,this,Qt::Dialog);
		msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
		switch (msgBox.exec()) 
		{
		case QMessageBox::Yes:
			playGround->activeRobot->startComms();
		    break;
		case QMessageBox::No:
		    return;
		   break;
		default:
		    return;
		    break;
		}			
	}
	if(playGround->activeRobot->notFollowing)
	{
		if(playGround->activeRobot->navigator->isRunning())
		{
			playGround->activeRobot->navigator->quit();
		}
		if(path)
		{
			playGround->activeRobot->navigator->setPath(path);
			playGround->activeRobot->navigator->start();
			pathFollowBtn.setText("Stop");
			playGround->activeRobot->notFollowing = false;
		}
	}
	else
	{
		if(playGround->activeRobot->navigator->isRunning())
		{
			playGround->activeRobot->navigator->StopNavigating();
			playGround->activeRobot->navigator->quit();	
			qDebug("Quitting Thread");
		}
		pathFollowBtn.setText("Path Follow");
		playGround->activeRobot->notFollowing = true;
	}
}

void NavControlPanel::updateSelectedRobot(bool)
{
	for(int i=0;i<availableRobots.size();i++)
	{
		if(availableRobots[i]->isChecked())
		{
			playGround->activeRobot = playGround->robotPlatforms[i];
//			qDebug("Seleted Robot is:%s",qPrintable(playGround->activeRobot->robot->robotName));	fflush(stdout);
//			if(!playGround->activeRobot)
//				return;
//			if(!playGround->activeRobot->planningManager)
//				return;		
//			if(!playGround->activeRobot->planningManager->pathPlanner)
//				playGround->activeRobot->startPlanner();
			return;
		}
	}
}

void NavControlPanel::setNavigation()
{
	if(playGround->activeRobot->notPaused)
	{
		playGround->activeRobot->navigator->setPause(true);
		playGround->activeRobot->notPaused = false;
		pauseBtn.setText("Continue");
	}
	else
	{
		playGround->activeRobot->navigator->setPause(false);		
		playGround->activeRobot->notPaused = true;
		pauseBtn.setText("Pause");		
	}
}

void NavControlPanel::pathPlan()
{
	playGround->activeRobot->planningManager->setMap(playGround->mapManager->globalMap);
	path = playGround->activeRobot->planningManager->findPath(METRIC);						
}

void NavControlPanel::generateSpace()
{
	playGround->activeRobot->planningManager->generateSpace();
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
	if(this->playGround->activeRobot)
	{
		playGround->activeRobot->planningManager->setStart(startLoc);
	}
	else
	{
		qDebug("No Robot is Selected");
	}
}

void NavControlPanel::setEnd(Pose endLoc)
{
	if(this->playGround->activeRobot)
	{
		playGround->activeRobot->planningManager->setEnd(endLoc);
	}
	else
	{
		qDebug("No Robot is Selected");
	}	
}

void NavControlPanel::setMap(Map * map)
{
	if(this->playGround->activeRobot)
	{
		playGround->activeRobot->planningManager->setMap(map);
	}
	else
	{
		qDebug("No Robot is Selected");
	}		
}

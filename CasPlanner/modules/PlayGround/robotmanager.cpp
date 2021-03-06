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
#include <robotmanager.h>

#include "commmanager.h"
#include "planningmanager.h"
#include "navigator.h"
#include "configfile.h"
#include "playground.h"
#include "IntentionRecognizer.h"
#include "socialplanner.h"

RobotManager::RobotManager():
        playGround(NULL),
        commManager(NULL),
        planningManager(NULL),
        navigator(NULL),
        intentionRecognizer(NULL),
        robot(NULL)
{
        // Empty Constructor
}

RobotManager::~RobotManager()
{
    if(commManager)
        delete commManager;
    if(navigator)
        delete navigator;
    if(robot)
        delete robot;
    if(planningManager)
        delete planningManager;
    if(socialPlanner)
        delete socialPlanner;
}
/* Read Configuration File and Initialize corresponding Sections :
 *  1- Initialize communication with the Player Server.
 *  2- Create the Specific GUI if required / not supported yet.
 *  3- Initialize Map management layer to provide the planner with maps.
 *  4- Initialize the Path Planning solver with the specified parameters.
 */
RobotManager::RobotManager(PlayGround *playG,ConfigFile *cf,int secId):
    playGround(playG),
    commManager(NULL),
    planningManager(NULL),
    navigator(NULL),
    intentionRecognizer(NULL),
    robot(NULL),
    socialPlanner(NULL)
{
    connect(playGround,SIGNAL(mapUpdated(Map *)),this,SLOT(updateMap(Map *)));
    connect(this,SIGNAL(addMsg(int,int,QString)),playGround,SLOT(addMsg(int,int,QString)));
    readRobotConfigs(cf,secId);
    readCommManagerConfigs(cf,secId);
    /* Social Robot Has to be called
     * After The Robot  readRobotConfigs
     */
    setupSocialPlanner();
    int numSections = cf->GetSectionCount();
    for(int i=0; i < numSections; i++)
    {
        QString sectionName = cf->GetSectionType(i);
        if(sectionName == "Navigator")
        {
            readNavigatorConfigs(cf);
        }
        if(sectionName == "Planner")
        {
            readPlannerConfigs(cf);
        }
    }
}

bool RobotManager::hasPath()
{
    if(planningManager->pathPlanner->path)
        return true;
    else
        return false;
}

bool RobotManager::isConnected()
{
    return commManager->isConnected();
}

bool RobotManager::isNavigating()
{
    return navigator->isRunning();
}

bool  RobotManager::isNavigationPaused()
{
    if(navigator)
        return navigator->isPaused();
    else
        return false;
}

void RobotManager::updateMap(Map * mapData)
{
    if(planningManager)
        planningManager->setMap(mapData);
    if(socialPlanner)
        socialPlanner->setMap(mapData);
}

void RobotManager::readRobotConfigs(ConfigFile *cf,int secId)
{
    QString logMsg;
    robot = new Robot();
    robot->readConfigs(cf,secId);
    logMsg.append("\n-> Robot Configurations Read.");
    logMsg.append("\n*********************************************************************");
    logMsg.append(QString("\n\t\t Robot  name:\t%1").arg(robot->robotName));
    logMsg.append(QString("\n\t\t Robot Ip is:\t%1:%2").arg(robot->robotIp).arg(robot->robotPort));
    logMsg.append("\n*********************************************************************");
    Q_EMIT addMsg(0,INFO,logMsg);
}

void RobotManager::readCommManagerConfigs(ConfigFile *cf,int secId)
{
    try
    {
        commManager = new CommManager(robot,this->playGround);
        commManager->readConfigs(cf,secId);
    }
    catch (CasPlannerException &e)
    {
        cout<< e.what();
    }
}

void RobotManager::readNavigatorConfigs(ConfigFile *cf)
{
    navigator = new Navigator(playGround,this);
    navigator->readConfigs(cf);
}

void RobotManager::readPlannerConfigs(ConfigFile *cf)
{
    planningManager = new PlanningManager(this);
    planningManager->readConfigs(cf);
}

void RobotManager::setPauseNavigation(bool pause)
{
    if(navigator)
        navigator->setPause(pause);
}

void RobotManager::start()
{
    startComms();
    startPlanner();
    startNavigator();
}

void RobotManager::stop()
{
    if(navigator)
    while(navigator->isRunning())
    {
        navigator->StopNavigating();
        sleep(0.1);
    }
    if(commManager)
        commManager->stop();
}

void RobotManager::stopNavigating()
{
    if(navigator)
    while(navigator->isRunning())
    {
        navigator->StopNavigating();
        sleep(0.1);
    }
}

void RobotManager::startComms()
{
    if(!commManager->isRunning())
    {
        commManager->start();
    }
}

void RobotManager::startPlanner()
{
    planningManager->setupPlanner();
}

void RobotManager::startNavigator()
{
    navigator->start();
}

void RobotManager::startNavigating()
{
    navigator->start();
}

void RobotManager::startIntentionRecognizer()
{
    intentionRecognizer = new IntentionRecognizer(this->playGround,this);
    intentionRecognizer->runRecognition = true;
    intentionRecognizer->start();
}

void RobotManager::setupSocialPlanner()
{
    socialPlanner = new SocialPlanner(this->playGround,this);
}

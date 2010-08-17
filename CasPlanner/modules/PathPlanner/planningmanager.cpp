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
#include "planningmanager.h"

/*!
 * Constructor, parameters are provided directly rather than
 * read from the configuration file
 */
PlanningManager::PlanningManager(RobotManager *robMan,
                                 double pixel_res,
                                 double dist_goal,
                                 double bridge_len,
                                 double bridge_res,
                                 double reg_grid,
                                 double obst_exp,
                                 double conn_rad,
                                 double obst_pen,
                                 double bridge_conn_rad
                                      )
{
    this->pixel_res  = pixel_res;
    this->bridge_len = bridge_len;
    this->bridge_res = bridge_res;
    this->reg_grid = reg_grid;
    this->obst_exp = obst_exp;
    this->reg_grid_conn_rad = conn_rad;
    this->obst_pen = obst_pen;
    this->dist_goal = dist_goal;
    this->bridge_conn_rad = bridge_conn_rad;
    this->pathPlanner = NULL;
    this->robotManager = robMan;
    this->connNodesEnabled      = CasPlanner::settings().isConnectNodesEnabled();
    this->regGridEnabled        = CasPlanner::settings().isRegGridEnabled();
    this->obstPenEnabled        = CasPlanner::settings().isObstaclePenEnabled();
    this->expObstEnabled        = CasPlanner::settings().isExpandObstEnabled();
    this->bridgeTestEnabled     = CasPlanner::settings().isBridgeTestEnabled();
    this->renderSearchSpaceTree = false;
    this->renderSearchTree      = false;
    this->renderPaths           = false;
    this->loadSpaceFromFile     = true;
    this->overWriteExistingSpace= false;
    planningParameters          = 0;

    if(connNodesEnabled)
        planningParameters|=NODES_CONNECT;
    if(regGridEnabled)
        planningParameters|=REGULAR_GRID;
    if(expObstEnabled)
        planningParameters|=OBST_EXPAND;
    if(obstPenEnabled)
        planningParameters|=OBST_PENALTY;
    if(bridgeTestEnabled)
        planningParameters|=BRIDGE_TEST;

    this->planningStep = WAITING;
    robotManager->robot->setCheckPoints(obst_exp);
    connect(this, SIGNAL(addMsg(int,int,QString)), robMan->playGround,SLOT(addMsg(int,int,QString)));
    this->setupPlanner();
}

PlanningManager::PlanningManager(RobotManager *robMan):
pathPlanner(NULL),
renderSearchSpaceTree(false),
renderSearchTree(false),
renderPaths(false),
robotManager(robMan),
loadSpaceFromFile(true),
overWriteExistingSpace(false),
planningParameters(0),
planningStep(WAITING)
{
    this->connNodesEnabled      = CasPlanner::settings().isConnectNodesEnabled();
    this->regGridEnabled        = CasPlanner::settings().isRegGridEnabled();
    this->obstPenEnabled        = CasPlanner::settings().isObstaclePenEnabled();
    this->expObstEnabled        = CasPlanner::settings().isExpandObstEnabled();
    this->bridgeTestEnabled     = CasPlanner::settings().isBridgeTestEnabled();
    connect(this, SIGNAL(addMsg(int,int,QString)), robMan->playGround,SLOT(addMsg(int,int,QString)));
    if(connNodesEnabled)
        planningParameters|=NODES_CONNECT;
    if(regGridEnabled)
        planningParameters|=REGULAR_GRID;
    if(expObstEnabled)
        planningParameters|=OBST_EXPAND;
    if(obstPenEnabled)
        planningParameters|=OBST_PENALTY;
    if(bridgeTestEnabled)
        planningParameters|=BRIDGE_TEST;
}

PlanningManager::~PlanningManager()
{

}

void PlanningManager::setRobotManager(RobotManager *rob)
{
    this->robotManager = rob;
}

void PlanningManager:: setBridgeTest(bool state)
{
    bridgeTestEnabled = state;
    if(state)
        planningParameters|=BRIDGE_TEST;
    else
        planningParameters&=(0x1F^BRIDGE_TEST);
}

void PlanningManager:: setConnNodes(bool state)
{
    connNodesEnabled = state;
    if(state)
        planningParameters^=NODES_CONNECT;
    else
        planningParameters&=(0x1F^NODES_CONNECT);
}

void PlanningManager:: setRegGrid(bool state)
{
    regGridEnabled = state;
    if(state)
        planningParameters^=REGULAR_GRID;
    else
        planningParameters&=(0x1F^REGULAR_GRID);
}

void PlanningManager:: setObstPen(bool state)
{
    obstPenEnabled = state;
    if(state)
        planningParameters^=OBST_PENALTY;
    else
        planningParameters&=(0x1F^OBST_PENALTY);
}

void PlanningManager:: setExpObst(bool state)
{
    expObstEnabled = state;
    if(state)
        planningParameters^=OBST_EXPAND;
    else
        planningParameters&=(0x1F^OBST_EXPAND);
}

void PlanningManager::setBridgeTestValue(double val)
{
    pathPlanner->setBridgeLen(val);
}

void PlanningManager::setConnNodesValue(double val )
{
    pathPlanner->setConRad(val);
}

void PlanningManager::setRegGridValue(double val)
{
    pathPlanner->setRegGrid(val);
}

void PlanningManager::setObstPenValue(double val)
{
    pathPlanner->setObstDist(val);
}

void PlanningManager::setExpObstValue(double val)
{
    pathPlanner->setExpRad(val);
}

void PlanningManager::setBridgeResValue(double val)
{
    pathPlanner->setBridgeRes(val);
}

void PlanningManager::setMap(Map * mapData)
{
    if(!this->pathPlanner)
        this->setupPlanner();
    pathPlanner->setMap(mapData);
}

void PlanningManager::setMap(LaserScan laserScan,double local_dist,Pose robotLocation)
{
    if(!this->pathPlanner)
        this->setupPlanner();
    pathPlanner->setMap(robotManager->playGround->mapManager->provideLaserOG(laserScan,local_dist,pixel_res,robotLocation));
}

void PlanningManager::updateMap(LaserScan laserScan,double local_dist,Pose robotLocation)
{
    Map *newMap = robotManager->playGround->mapManager->providePointCloud(laserScan,local_dist,robotLocation);
    if(!this->pathPlanner)
        this->setupPlanner();
    pathPlanner->updateMap(newMap);
}

void PlanningManager::setStart(Pose start)
{
    this->start = start;
}

void PlanningManager::setEnd(Pose end)
{
    this->end = end;
}

bool PlanningManager::fileExist(const char * fname)
{
    struct stat stat_buf;
    if (stat(fname,&stat_buf) != 0)
        return false;
    return (stat_buf.st_mode & S_IFMT) == S_IFREG;
}

void PlanningManager::generateSpace()
{
    planningStep = GENERATING_SPACE;
    QThread::start();
}

void PlanningManager::findPath(int coord)
{
    this->coord  = coord;
    planningStep = FINDING_PATH;
    QThread::start();
}

int PlanningManager::readConfigs( ConfigFile *cf)
{
    int numSec;
    numSec = cf->GetSectionCount();
    for(int i=0; i < numSec; i++)
    {
        QString sectionName = cf->GetSectionType(i);
        if(sectionName == "Planner")
        {
            bridge_len =			cf->ReadFloat(i, "bridge_len",2);
            bridge_res = 			cf->ReadFloat(i, "bridge_res",0.5);
            reg_grid =				cf->ReadFloat(i, "reg_grid",0.5);
            obst_exp = 				cf->ReadFloat(i, "obst_exp",0.2);
            reg_grid_conn_rad =                 cf->ReadFloat(i, "reg_grid_conn_rad",0.8);
            obst_pen = 				cf->ReadFloat(i, "obst_pen",3);
            dist_goal = 			cf->ReadFloat(i, "dist_goal",0.2);
            bridge_conn_rad =                   cf->ReadFloat(i, "bridge_conn_rad",0.5);
        }
        if(sectionName == "Map")
        {
            pixel_res =  			cf->ReadFloat(i, "pixel_res",0.05);
            negate = 				cf->ReadInt(i, "negate",0);
        }
    }
    robotManager->robot->setCheckPoints(obst_exp);
    this->setupPlanner();
    return 1;
}

int PlanningManager::setupPlanner()
{
    QString logMsg;
    if (!pathPlanner)
    {
        logMsg.append("\n-> Starting Planner.");
        logMsg.append("\n\tPlanning Parameters:");
        logMsg.append(QString("\n\t\t\t Pixel Resolution = %1").arg(pixel_res));
        logMsg.append(QString("\n\t\t\t Distance to Goal = %1").arg(dist_goal));
        logMsg.append(QString("\n\t\t\t Bridge Test Lenght = %1").arg(bridge_len));
        logMsg.append(QString("\n\t\t\t Bridge Test Res = %1").arg(bridge_res));
        logMsg.append(QString("\n\t\t\t Bridge Conn Rad = %1").arg(bridge_conn_rad));
        logMsg.append(QString("\n\t\t\t Reg Grid Res  = %1").arg(reg_grid));
        logMsg.append(QString("\n\t\t\t Obstacle Expansion Radius = %1").arg(obst_exp));
        logMsg.append(QString("\n\t\t\t Connection Radius = %1").arg(reg_grid_conn_rad));
        logMsg.append(QString("\n\t\t\t Obstacle Penalty = %1").arg(obst_pen));

        pathPlanner = new PathPlanner(robotManager->robot,
                                      dist_goal,
                                      bridge_len,
                                      bridge_res,
                                      reg_grid,
                                      reg_grid_conn_rad,
                                      obst_pen,
                                      bridge_conn_rad);
        logMsg.append("\n->Planner Started.");
        LOG(Logger::Info,logMsg)
        emit addMsg(0,INFO,logMsg);
    }
    return 1;
}

void PlanningManager::generateSearchSpace(bool loadFromFile,bool overWriteCurrent)
{
    loadSpaceFromFile        = loadFromFile;
    overWriteExistingSpace   = overWriteCurrent;
    planningStep             = GENERATING_SPACE;
    QThread::start();
}

void PlanningManager::generateSearchSpaceState(bool loadFromFile,bool overWriteCurrent)
{
    QTime timer;
    const char * filename = "logs/SearchSpace.txt";
    if(!this->pathPlanner)
        this->setupPlanner();
    if(pathPlanner->search_space && !overWriteCurrent)
    {
        LOG(Logger::Warning,"Search Space already Exist")
        return;
    }
    if(overWriteCurrent)
    {
        pathPlanner->freeResources();
    }
    timer.restart();
    if(fileExist(filename) && loadFromFile)
    {
        LOG(Logger::Info,"Loading Space From file ...")
        if(pathPlanner->readSpaceFromFile(filename))
        {
            if(expObstEnabled)
                pathPlanner->expandObstacles();
            if(connNodesEnabled)
                pathPlanner->connectNodes();
            LOG(Logger::Info,"File loading took:"<<timer.elapsed()/double(1000.00)<<" secs")
        }
        else
        {
            LOG(Logger::Warning,"Could not Load Search Space from File")
        }
    }
    else
    {
        LOG(Logger::Info,"Generating Space ...")
        if(planningParameters & OBST_EXPAND)
            pathPlanner->expandObstacles();
        if(planningParameters & REGULAR_GRID)
            pathPlanner->generateRegularGrid();
        if(planningParameters & BRIDGE_TEST)
            pathPlanner->bridgeTest();
        if(planningParameters & OBST_PENALTY)
            pathPlanner->addCostToNodes();
        if(planningParameters & NODES_CONNECT)
            pathPlanner->connectNodes();
        pathPlanner->saveSpace2File(filename);
        emit searchSpaceGenerated();
        LOG(Logger::Info,"Space Generation took:"<<timer.elapsed()/double(1000.00)<<" secs")
    }
    pathPlanner->showConnections();
    this->loadSpaceFromFile     = true;
    this->overWriteExistingSpace= false;
}

void PlanningManager::findPathState()
{
    if(!this->pathPlanner)
        this->setupPlanner();
    Node * retval;
    if(!pathPlanner->search_space || planningParameters!=pathPlanner->getPlanningParameters())
    {
        generateSearchSpaceState();
    }
    QTime timer;
    retval = pathPlanner->startSearch(start,end,coord);
    LOG(Logger::Info,"File loading took:"<<timer.elapsed()/double(1000.00)<<" secs")
    emit pathFound(retval);
    if(retval)
    {
        //pathPlanner->printNodeList();
    }
    else
    {
        LOG(Logger::Info,"No Path Found")
    }
}

void PlanningManager::run()
{
    switch(planningStep)
    {
    case GENERATING_SPACE:
        generateSearchSpaceState(loadSpaceFromFile,overWriteExistingSpace);
        break;
    case FINDING_PATH:
        findPathState();
        break;
    case WAITING:
        break;
    }
}

int PlanningManager::stop()
{
    return 1;
}


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
#include "playground.h"

PlayGround::PlayGround():
    navCon(NULL),
    mapViewer(NULL),
    mapManager(NULL)
{

}

PlayGround::PlayGround(QStringList configFiles,QStatusBar *in_statusBar):
    navCon(NULL),
    mapViewer(NULL),
    mapManager(NULL),
    activeRobot(NULL)
{
    statusLogger = new StatusLogger(in_statusBar);
    for(int j=0; j < configFiles.size(); j++)
    {
        ConfigFile *cf = new ConfigFile("localhost",6665);
        XmlParser  *xmlParser = NULL;
        if(configFiles[j].contains(".xml", Qt::CaseInsensitive))
        {
            xmlParser = new XmlParser(configFiles[j]);
            xmlParser->read();
        }
        int numSections;
        cf->Load(configFiles[j].toLocal8Bit());
        numSections = cf->GetSectionCount();
        for(int i=0; i < numSections; i++)
        {
            QString sectionName = cf->GetSectionType(i);
            if(sectionName == "GUI")
            {
                QString render = cf->ReadString(i, "renderingMethod", "OpenGL");
                if(render =="OpenGL")
                {

                }
            }
            if(sectionName == "Map")
            {
                QString mapName = cf->ReadString(i, "mapname", "resources//casareaicpB.png");
                float mapRes  =   cf->ReadFloat(i, "pixel_res",0.05);
                bool mapNegate  = cf->ReadBool(i, "negate",0);
                loadMap(mapName,mapRes,mapNegate,Pose(0,0,0));
            }
            if(sectionName == "Robot")
            {
                RobotManager *rbm = new RobotManager(this,cf,i);
                robotPlatforms.push_back(rbm);
            }
        }
        delete cf;
        if(xmlParser)
            delete xmlParser;
    }
}

void PlayGround::addMsg(int id,int type,QString msg)
{
        statusLogger->addStatusMsg(id,type,msg);
}

void PlayGround::loadMap(QString name,float res,bool negate,Pose p)
{
    QString logMsg;
    logMsg.append("\nStarting Map Manager");
    if(!mapManager)
        mapManager = new MapManager(name,res,negate,p);
    else
        mapManager->loadMap(name,res,negate,p);
    logMsg.append("\nMap Manager Started");
    addMsg(0,INFO,logMsg);
    emit mapUpdated(mapManager->globalMap);
}

void PlayGround::setMapViewer(MapViewer *_mapViewer)
{
    this->mapViewer = _mapViewer;
    for(int i=0;i<robotPlatforms.size();i++)
        connect(robotPlatforms[i]->planningManager,SIGNAL(searchSpaceGenerated()),mapViewer,SLOT(searchSpaceGenerated()));
}

int PlayGround::setNavContainer(NavContainer* con)
{
    this->navCon = con;
    return 1;
}

void PlayGround::startRobotsComm()
{
    for(int i=0;i<robotPlatforms.size();i++)
        robotPlatforms[i]->startComms();
}

void PlayGround::stopRobots()
{
    for(int i=0;i<robotPlatforms.size();i++)
        robotPlatforms[i]->stop();
}

PlayGround::~PlayGround()
{
    for(int i=0;i<robotPlatforms.size();i++)
        delete robotPlatforms[i];
}

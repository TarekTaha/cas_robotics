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
#include "tabcontainer.h"
#include "playgroundtab.h"
#include "playground.h"
#include "tasksgui.h"
#include "mapviewer.h"
#include "hritab.h"
#include "loggerview.h"
#include "missioncontroltab.h"
#include "logger.h"

TabContainer::TabContainer(QWidget *parent,PlayGround *playG)
    : QTabWidget(parent),
      playGround(playG)
{
    missionControlTab = new MissionControlTab(parent,playGround);
    addTab(missionControlTab,"Mission Control Tab");

    tasksGui = new TasksGui(parent,playGround);
    addTab(tasksGui, "Tasks Manager");

    playGroundTab = new PlayGroundTab(parent,playGround);
    addTab(playGroundTab, "PlayGround");

    hriTab = new HriTab(parent,playGround);
    addTab(hriTab, "HRI Settings");

    loggerView = new loggerview(parent);
    addTab(loggerView,"Logger");

    //connect(&Logger::getLogger(),SIGNAL(showMsg(int,QString)),loggerView,SLOT(showMsg(int,QString)));
    //LOG(Logger::Info,"TEST,TEST,TEST")

    updateGeometry();
}

void TabContainer::setPlayGround(PlayGround *playG)
{
        playGround = playG;
}

TabContainer::~TabContainer()
{
}



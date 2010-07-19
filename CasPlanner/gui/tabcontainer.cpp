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

TabContainer::TabContainer(QWidget *parent,PlayGround *playG)
    : QTabWidget(parent),
      playGround(playG)
{
    navCon = 	new NavContainer(parent,playGround);
    addTab(navCon, "Navigation Panel");

    tasksGui = new TasksGui(parent,playGround);
    addTab(tasksGui, "Tasks Manager");

    playGroundTab = new PlayGroundTab(parent,playGround);
    addTab(playGroundTab, "PlayGround");

    planningTab = new PlanningTab(parent,playGround);
    addTab(planningTab, "Path Planning");

    hriTab = new HriTab(parent,playGround);
    addTab(hriTab, "HRI Settings");

    loggerView = new loggerview(parent);
    addTab(loggerView,"Logger");

    Logger& logger = Logger::getLogger();
    connect(&logger,SIGNAL(logMsg(int,QString)),loggerView,SLOT(logMsg(int,QString)));
    updateGeometry();
}

void TabContainer::setPlayGround(PlayGround *playG)
{
        playGround = playG;
}

TabContainer::~TabContainer()
{
}



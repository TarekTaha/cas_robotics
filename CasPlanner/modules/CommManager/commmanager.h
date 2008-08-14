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
#ifndef COMMMANAGER_H
#define COMMMANAGER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QObject>

#include "playerinterface.h"
#include "robot.h"
#include "statusbar.h"
#include "playground.h"

class PlayGround;

class CommManager: public PlayerInterface
{
    Q_OBJECT
    public:
        CommManager(Robot *,PlayGround * playG);
        CommManager();
        ~CommManager(); 
        int readConfigs(ConfigFile *cf,int secId);
        int initialize(); 
    protected:
        Robot * robot;
       	PlayGround *playGround;
};

#endif


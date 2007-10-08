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
#ifndef OGRENDER_H
#define OGRENDER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include "glrender.h"
#include "interfaceprovider.h"

class OGRenderer: public GLRender 
{
Q_OBJECT
    public: 
        OGRenderer(QGLWidget *w);
        ~OGRenderer(); 
        virtual void setId(int mapId);
        virtual void setProvider(MapProvider *provider);
        virtual void render();
    public slots:
        virtual void updateData(); 
    private:
        double maxRange; 
        int mapId; 
        MapProvider *provider; 
        Map mapData; 
};

#endif


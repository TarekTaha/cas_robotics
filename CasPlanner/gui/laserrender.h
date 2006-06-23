/***************************************************************************
 *   Copyright (C) 2006 by Waleed Kadous   *
 *   waleed@width   *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef LASERRENDER_H
#define LASERRENDER_H

#include "glrender.h"
#include "interfaceprovider.h"

class LaserRender: public GLRender 
{
Q_OBJECT
    public: 
        LaserRender(QGLWidget *w);
        ~LaserRender(); 
        virtual void setId(int laserId);
        virtual void setProvider(LaserProvider *provider);  
        virtual void render(); 
        virtual void setRange(double range); 
    public slots:
        virtual void updateData(); 
    private:
        double maxRange; 
        int laserId; 
        LaserProvider *provider; 
        QVector<QPointF> laserData; 
};

#endif


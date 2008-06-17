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
#include "laserrender.h"

LaserRender::LaserRender(QGLWidget *in_w): 
  GLRender(in_w),
  maxRange(8),
  laserId(0),
  provider(0)
{

}

LaserRender::~LaserRender(){};

void LaserRender::setId(int id)
{
    laserId = id;  
}

void LaserRender::setProvider( LaserProvider *i_prov)
{
    provider = i_prov;
}

void LaserRender::updateData()
{
      provider->getLaserScan(laserData);
      w->updateGL(); 
}

void LaserRender::setRange(double range)
{
    maxRange = range;  
}

void LaserRender::render()
{
    w->makeCurrent(); 
    glPushMatrix(); 
    glTranslatef(0.5, 0.5, 0);
    glScalef(0.5/maxRange, 0.5/maxRange, 1.0); 
    glColor4f(1.0,1.0,1, 1);
    
    glBegin(GL_QUADS); 
    	glVertex2f(-maxRange, -maxRange); 
	    glVertex2f(-maxRange, maxRange); 
	    glVertex2f(maxRange, maxRange); 
    	glVertex2f(maxRange, -maxRange); 
//    	glVertex2f(0, 0); 
//	    glVertex2f(0, 1); 
//	    glVertex2f(1, 1); 
//    	glVertex2f(1, 0); 
    glEnd(); 
    
    glColor3f(0.623,0.811,1); 
    
    glBegin(GL_TRIANGLE_FAN);
    	glVertex2f(0,0);  
	    if(laserData.points.size() > 0)
    	{
        	for(int i=0; i < laserData.points.size(); i++)
	        {
    	        glVertex2f(laserData.points[i].x(), laserData.points[i].y());  
        	}
	    }
    	glVertex2f(0,0);
    glEnd(); 
    glPopMatrix();
}

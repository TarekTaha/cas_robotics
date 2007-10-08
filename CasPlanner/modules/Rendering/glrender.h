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
#ifndef GLRENDER_H
#define GLRENDER_H
#include <QtOpenGL> 

// Basic assumptions: Objects render themselves in an area that extends from (0,0) to (1,1) 
class GLRender: public QObject 
{
Q_OBJECT
    public: 
        GLRender(QGLWidget *in_w): w(in_w){}; 
        virtual void render()=0; 
        virtual ~GLRender()
        {
        }
    public slots:
        virtual void updateData()=0; 
    protected:
        QGLWidget *w; 
};

#endif

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
#include "ogrender.h"

OGRenderer::OGRenderer(QGLWidget *in_w): 
  GLRender(in_w),
  maxRange(4),
  mapId(0),
  provider(0)
{

}

OGRenderer::~OGRenderer(){};

void OGRenderer::setId(int id)
{
    qDebug("Map id set to %d", id); 
    mapId = id;  
}

void OGRenderer::setProvider( MapProvider *prov)
{
    provider = prov;
}

void OGRenderer::updateData()
{
      mapData = provider->provideMap();
      w->updateGL();
}

void OGRenderer::render()
{
//    qDebug("Map width %d height %d resolution %f" , mapData.width, mapData.height, mapData.resolution);
//    if(mapData.width == 0 || mapData.height || mapData.resolution==0)
//    	return;
    unsigned char imgData[mapData.width*mapData.height*4];
    int min=255; 
    int max=0; 
    float mean=0; 
    w->makeCurrent(); 
    glPushMatrix(); 
    glTranslatef(0,0,0); 
    glScalef(0.5/mapData.width, 0.5/mapData.width, 1.0); 
    //glScalef(1,1, 1.0); 
    glColor4f(1.0,1.0,1, 1);
    glBegin(GL_QUADS); 
    glVertex2f(-mapData.width, -mapData.width); 
    glVertex2f(-mapData.width, mapData.width); 
    glVertex2f(mapData.width, mapData.width); 
    glVertex2f(mapData.width, -mapData.width); 
    glEnd(); 
//    glColor3f(0.623,0.811,1); 
    glBegin(GL_POINTS);
    for(int i=0; i < mapData.height; i++)
    {
		for(int j=0; j < mapData.width; j++)
		{
		    // qDebug("Pixel %d has value %d", k, (unsigned char) mapData.data[k]);
		    unsigned char val = (unsigned char) mapData.rawData[i*mapData.width+j];
		    if(val < min) min=val;
		    if(val > max) max=val;
		    mean += val;
		    //qDebug("\n VAL:%u",val);
	    	glColor3f(val/255,val/255,val/255); 
	    	glVertex2f(i,j);		    
		    if(val > 90)
		    {
		    	//qDebug("Pixel Occupied val=%u",val);
				imgData[(i*mapData.width+j)*4] = 0;
				imgData[(i*mapData.width+j)*4+1] =0; 
				imgData[(i*mapData.width+j)*4+2] = 255; 
				imgData[(i*mapData.width+j)*4+3] = 255;
		    }
		    else if(val < 70)
		    {
//				if(!highlighted)
//				{
//				    imgData[(i*mapData.width+j)*4] = 255;
//				    imgData[(i*mapData.width+j)*4+1] =255; 
//				    imgData[(i*mapData.width+j)*4+2] = 255; 
//				    imgData[(i*mapData.width+j)*4+3] = 50;
//				}
//				else 
				{
				    imgData[(i*mapData.width+j)*4] = 255;
				    imgData[(i*mapData.width+j)*4+1] =255; 
				    imgData[(i*mapData.width+j)*4+2] = 0; 
				    imgData[(i*mapData.width+j)*4+3] = 100;
				}
		    }
		    else 
		    {
				imgData[(i*mapData.width+j)*4] = 0;  
				imgData[(i*mapData.width+j)*4+1] = 0; 
				imgData[(i*mapData.width+j)*4+2] = 0;  
				imgData[(i*mapData.width+j)*4+3] = 0;   
		    }
		}
    }
    glEnd(); 
    glPopMatrix();
//    //qDebug("Map values %d %d %f" , min, max, mean/(mapData.width*mapData.height));
//    w->makeCurrent(); 
//   	GLuint texId; 
//    //setFocusPolicy(Qt::StrongFocus);
//    glGenTextures(1, &texId); 
//    glBindTexture(GL_TEXTURE_2D, texId);
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, mapData.width, mapData.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, imgData);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR);
//    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
//    glEnable(GL_TEXTURE_2D);
//    glPushMatrix();
//    
//    glScalef(1,-1,1);
//    glTranslatef(-mapData.width*mapData.resolution/2.0,-mapData.height*mapData.resolution/2,0);
//    glColor4f(0,0,1,0.8);
//    glShadeModel(GL_FLAT);
// 
//    glBegin(GL_QUADS);
//    glTexCoord2f(0.0,0.0);
//    glVertex2f(0.0,0.0);
//    glTexCoord2f(0.0,1);
//    glVertex2f(0.0, mapData.height*mapData.width);
//    glTexCoord2f(1,1);
//    glVertex2f(mapData.resolution*mapData.width,mapData.height*mapData.resolution);
//    glTexCoord2f(1,0.0);
//    glVertex2f(mapData.width*mapData.resolution,0.0);
//    glEnd();
// 
//    glDisable(GL_TEXTURE_2D);
//    
//    // We now draw a surrounding rectangle so people know where the edge of the OG map is. 
//    //if(showPatchBorders)
////    {
////		glColor4f(0,0,1,0.5); 
////		glBegin(GL_LINE_LOOP); 
////		glVertex2f(0,0);
////		glVertex2f(0.0, mapData.height*mapData.resolution);
////		glVertex2f(mapData.width*mapData.resolution,mapData.height*mapData.resolution);
////		glVertex2f(mapData.width*mapData.resolution,0.0);
////		glEnd();
////    } 
//	glPopMatrix();
}

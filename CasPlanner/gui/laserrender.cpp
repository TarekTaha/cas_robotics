#include "laserrender.h"

LaserRender::LaserRender(QGLWidget *in_w): 
  GLRender(in_w),
  maxRange(4),
  laserId(0),
  provider(0)
{

}

LaserRender::~LaserRender(){};

void LaserRender::setId(int id)
{
    qDebug("Laser set to %d", id); 
    laserId = id;  
}

void LaserRender::setProvider( LaserProvider *i_prov)
{
    provider = i_prov;
}

void LaserRender::updateData()
{
      laserData = provider->getLaserScan(laserId);
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
    //glTranslatef(0.5, 0.5, 0);
    //glScalef(0.5/maxRange, 0.5/maxRange, 1.0); 
    glColor4f(1.0,1.0,1, 1);
    
    glBegin(GL_QUADS); 
//    	glVertex2f(-maxRange, -maxRange); 
//	    glVertex2f(-maxRange, maxRange); 
//	    glVertex2f(maxRange, maxRange); 
//    	glVertex2f(maxRange, -maxRange); 
    	glVertex2f(0, 0); 
	    glVertex2f(0, 1); 
	    glVertex2f(1, 1); 
    	glVertex2f(1, 0); 
    glEnd(); 
    
    glColor3f(0.623,0.811,1); 
    
    glBegin(GL_TRIANGLE_FAN);
    	glVertex2f(0,0);  
	    if(laserData.size() > 0)
    	{
        	for(int i=0; i < laserData.size(); i++)
	        {
    	        glVertex2f(laserData[i].x(), laserData[i].y());  
        	}
	    }
    	glVertex2f(0,0);
    glEnd(); 
    glPopMatrix();
}

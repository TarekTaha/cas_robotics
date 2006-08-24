#include "robotrender.h"

RobotRender::RobotRender(QGLWidget *in_w,Robot *rob): 
  GLRender(in_w),
  robot(rob)
{

}

RobotRender::~RobotRender(){};

void RobotRender::setRobot(Robot *rob)
{
	this->robot= rob;
}
void RobotRender::render()
{
    w->makeCurrent(); 
    glPushMatrix(); 
//    glTranslatef(0.5, 0.5, 0);
//    glScalef(0.5/maxRange, 0.5/maxRange, 1.0); 
//    glColor4f(1.0,1.0,1, 1);
//    
//    glBegin(GL_QUADS); 
//    	glVertex2f(-maxRange, -maxRange); 
//	    glVertex2f(-maxRange, maxRange); 
//	    glVertex2f(maxRange, maxRange); 
//    	glVertex2f(maxRange, -maxRange); 
//    glEnd(); 
//    
//    glColor3f(0.623,0.811,1); 
//    
//    glBegin(GL_TRIANGLE_FAN);
//    	glVertex2f(0,0);  
//	    if(laserData.size() > 0)
//    	{
//        	for(int i=0; i < laserData.size(); i++)
//	        {
//    	        glVertex2f(laserData[i].x(), laserData[i].y());  
//        	}
//	    }
//    	glVertex2f(0,0);
//    glEnd(); 
    glPopMatrix();
}

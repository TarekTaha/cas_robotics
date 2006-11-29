#include "speedrender.h"

SpeedRender::SpeedRender(QGLWidget *w):
    GLRender(w), maxSpeed(1), maxTurnRate(1)
{

}

void SpeedRender::setSpeedProvider(SpeedProvider *provider)
{
    //qDebug("Speed provider set"); 
    sp = provider;  
}

void SpeedRender::updateData()
{
    //qDebug("Data updated to %f %f %f %f", speed, turnRate, maxSpeed, maxTurnRate);
    sp->provideSpeed(speed, turnRate);
    w->updateGL(); 
}

void SpeedRender::setMaxSpeed(double speed)
{
    maxSpeed = speed;  
}

void SpeedRender::setMaxTurnRate(double turnRate)
{
    maxTurnRate = turnRate;  
}

void SpeedRender::render()
{
    
    float speedFraction = speed/maxSpeed;
    float turnFraction = turnRate/maxTurnRate; 
    //qDebug("Speed is %f TR is %f", speedFraction, turnFraction); 
    glColor4f(0,1,0,0.5); 
    glBegin(GL_LINE_LOOP);
    glVertex2f(0,0.5);
    glVertex2f(1,0.5); 
    glVertex2f(1,0.75); 
    glVertex2f(0, 0.75); 
    glEnd(); 
    glColor4f(1,0,0,0.5); 
    glBegin(GL_QUADS);
    glVertex2f(0.02,0.52); 
    glVertex2f(0.02+speedFraction*0.96, 0.52); 
    glVertex2f(0.02+speedFraction*0.96, 0.72); 
    glVertex2f(0.02,0.72); 
    glEnd(); 
    glColor4f(0,1,0,0.5); 
    glBegin(GL_LINES); 
    glVertex2f(0.5-turnFraction*0.5, 0.8); 
    glVertex2f(0.5+turnFraction*0.5, 0.8); 
    glEnd(); 
}

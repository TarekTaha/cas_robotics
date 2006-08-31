#ifndef LASERRENDER_H
#define LASERRENDER_H

#include "glrender.h"
#include "Robot.h"

class RobotRender: public GLRender 
{
	
Q_OBJECT
    public: 
        RobotRender(QGLWidget *w,Robot *rob);
        ~RobotRender(); 
        virtual void setRobot(Robot *);
        virtual void render(); 
//    public slots:
//        virtual void updateData(); 
    private:
        Robot *robot;
};

#endif


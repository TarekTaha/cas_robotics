#ifndef SPEEDRENDER_H
#define SPEEDRENDER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include "glrender.h"
#include "interfaceprovider.h"

class SpeedRender: public GLRender 
{
    public:
        SpeedRender(QGLWidget *w);
        void setSpeedProvider(SpeedProvider *sp);
        void setMaxSpeed(double speed);
        void setMaxTurnRate(double turnRate); 
        void render();
    public slots:
        void updateData(); 
    private:
        SpeedProvider *sp; 
        double speed; 
        double maxSpeed; 
        double turnRate; 
        double maxTurnRate; 
    
    
};

#endif 



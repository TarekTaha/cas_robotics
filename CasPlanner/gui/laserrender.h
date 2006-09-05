#ifndef LASERRENDER_H
#define LASERRENDER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

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
        LaserScan laserData; 
};

#endif


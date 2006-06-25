#ifndef OGRENDER_H
#define OGRENDER_H

#include "glrender.h"
#include "interfaceprovider.h"

class OGRenderer: public GLRender 
{
Q_OBJECT
    public: 
        OGRenderer(QGLWidget *w);
        ~OGRenderer(); 
        virtual void setId(int mapId);
        virtual void setProvider(MapProvider *provider);
        virtual void render();
    public slots:
        virtual void updateData(); 
    private:
        double maxRange; 
        int mapId; 
        MapProvider *provider; 
        Map mapData; 
};

#endif


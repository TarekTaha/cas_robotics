#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent,PlayGround *playG)
    : QTabWidget(parent),
      playGround(playG)
{
    navCon = 	new NavContainer(parent,playG);
    //mapViewer = new MapViewer(parent,rob);
    //sensorsGui = new SensorsGui(parent,rob);
    if(playG->renderingMethod == PAINTER_2D)
    {
    	addTab(navCon, "2D Painter Navigation Panel"); 
    }
    else
    {
    	addTab(navCon, "OpenGL Navigation Panel");     	
    }
    //addTab(mapViewer, "OpenGl Map Viewer"); 
    //addTab(sensorsGui, "Sensors Pannel");  
    updateGeometry();
}
void TabContainer::setPlayGround(PlayGround *playG)
{
	playGround = playG;
}

TabContainer::~TabContainer()
{
}



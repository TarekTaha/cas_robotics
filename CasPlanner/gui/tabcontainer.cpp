#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent,RobotManager *rob)
    : QTabWidget(parent),
      robotManager(rob)
{
    navCon = 	new NavContainer(parent,rob);
    //mapViewer = new MapViewer(parent,rob);
    //sensorsGui = new SensorsGui(parent,rob);
    if(rob->renderingMethod == PAINTER_2D)
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
void TabContainer::setRobotManager(RobotManager *robManager)
{
	robotManager = robManager;
}

TabContainer::~TabContainer()
{
}



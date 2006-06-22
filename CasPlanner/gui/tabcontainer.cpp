#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent)
    : QTabWidget(parent),
      navCon(parent),
      sensorsGui(comManager,parent)
{
    addTab(&navCon, "Navigation Panel"); 
    addTab(&sensorsGui, "Interactivity Pannel");     
    setTabIcon(1, QIcon(":warning.png")); 
    updateGeometry();
}

//void TabContainer::setMapManager(QTMapDataInterface *mapManager)
//{
//    mapEd.setMapManager(mapManager);  
//    snapView.setMapManager(mapManager); 
//}

void TabContainer::setCommManager(CommManager *comManager)
{
//	sensorGui.setRobotComms(comManager);
}

TabContainer::~TabContainer()
{
}



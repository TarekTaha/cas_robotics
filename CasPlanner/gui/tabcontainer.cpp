#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent)
    : QTabWidget(parent),
      senCon(parent),
      navCon(parent)
{
    addTab(&navCon, "Navigation Panel"); 
    addTab(&senCon, "Sensors Viewer"); 
    setTabIcon(1, QIcon(":warning.png")); 
    updateGeometry();
}

//void TabContainer::setMapManager(QTMapDataInterface *mapManager)
//{
//    mapEd.setMapManager(mapManager);  
//    snapView.setMapManager(mapManager); 
//}

TabContainer::~TabContainer()
{
}



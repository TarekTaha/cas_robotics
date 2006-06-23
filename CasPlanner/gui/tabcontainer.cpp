#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent)
    : QTabWidget(parent),
      navCon(parent),
      sensorsGui(robotManager,parent)
{
    addTab(&navCon, "Navigation Panel"); 
    addTab(&sensorsGui, "Interactivity Pannel");     
    setTabIcon(1, QIcon(":warning.png")); 
    updateGeometry();
}
void TabContainer::setRobotManager(RobotManager *robManager)
{
	robotManager = robManager;
}

TabContainer::~TabContainer()
{
}



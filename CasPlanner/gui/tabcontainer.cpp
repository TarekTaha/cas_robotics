#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent,RobotManager *rob)
    : QTabWidget(parent),
      robotManager(rob)
{
    navCon = new NavContainer(parent,rob);
    //sensorsGui = new SensorsGui(parent,rob);
    addTab(navCon, "Navigation Panel"); 
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



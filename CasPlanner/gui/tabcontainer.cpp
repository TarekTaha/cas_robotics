#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent,RobotManager *rob)
    : QTabWidget(parent),
      robotManager(rob),
      navCon(parent,rob),
      sensorsGui(parent,rob)
{
    addTab(&navCon, "Navigation Panel"); 
    addTab(&sensorsGui, "Sensors Pannel");     
    updateGeometry();
}
void TabContainer::setRobotManager(RobotManager *robManager)
{
	robotManager = robManager;
}

TabContainer::~TabContainer()
{
}



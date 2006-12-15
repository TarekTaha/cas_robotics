#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent,PlayGround *playG)
    : QTabWidget(parent),
      playGround(playG)
{
    navCon = 	new NavContainer(parent,playGround);
   	addTab(navCon, "Navigation Panel");
   	
    tasksGui = new TasksGui(parent,playGround);
    addTab(tasksGui, "Tasks Manager");
    
	playGroundTab = new PlayGroundTab(parent,playGround);    
    addTab(playGroundTab, "PlayGround");

    updateGeometry();
}
void TabContainer::setPlayGround(PlayGround *playG)
{
	playGround = playG;
}

TabContainer::~TabContainer()
{
}



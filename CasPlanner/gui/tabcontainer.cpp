#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent,PlayGround *playG)
    : QTabWidget(parent),
      playGround(playG)
{
    tasksGui = new TasksGui(parent,playGround);
    addTab(tasksGui, "Tasks Manager");
    	
    navCon = 	new NavContainer(parent,playGround);
   	addTab(navCon, "Navigation Panel");
   	  
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



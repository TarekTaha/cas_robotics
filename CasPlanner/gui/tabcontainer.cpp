#include "tabcontainer.h"

TabContainer::TabContainer(QWidget *parent,PlayGround *playG)
    : QTabWidget(parent),
      playGround(playG)
{
//     navCon = 	new NavContainer(parent,playGround);
    tasksGui = new TasksGui(parent,playGround);
	playGroundTab = new PlayGroundTab(parent,playGround);    
//    if(playG->renderingMethod == PAINTER_2D)
//    {
//    	addTab(navCon, "2D Painter Navigation Panel");
//    }
//    else
//    {
//     	addTab(navCon, "OpenGL Navigation Panel");
//    }
    addTab(tasksGui, "Tasks Manager");
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



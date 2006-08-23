#ifndef TABCONTAINER_H
#define TABCONTAINER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QTabWidget>
#include "sensorsgui.h"
#include "navigationtab.h"
#include "mapviewer.h"
class TabContainer : public QTabWidget
{
Q_OBJECT
    public:
		TabContainer(QWidget *parent=0,RobotManager *rob=0);
		void setRobotManager(RobotManager *robotManager); 
		~TabContainer();
    public:
	    RobotManager *robotManager;
		NavContainer *navCon;
		SensorsGui *sensorsGui;
		MapViewer *mapViewer;
};

#endif

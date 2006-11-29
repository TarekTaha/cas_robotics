#ifndef TABCONTAINER_H
#define TABCONTAINER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QTabWidget>
#include "playground.h"
#include "tasksgui.h"
#include "navigationtab.h"
#include "mapviewer.h"

class TabContainer : public QTabWidget
{
Q_OBJECT
    public:
		TabContainer(QWidget *parent=0,PlayGround *rob=0);
		void setPlayGround(PlayGround *playG);
		~TabContainer();
    public:
	    PlayGround   *playGround;
		NavContainer *navCon;
		TasksGui *tasksGui;
		MapViewer *mapViewer;
};

#endif

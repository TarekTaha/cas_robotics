#ifndef TABCONTAINER_H
#define TABCONTAINER_H
#include <QTabWidget>
#include "sensorsgui.h"
#include "navigationtab.h"
class TabContainer : public QTabWidget
{
Q_OBJECT
    public:
		TabContainer(QWidget *parent=0,RobotManager *rob=0);
		void setRobotManager(RobotManager *robotManager); 
		~TabContainer();
    private:
	    RobotManager *robotManager;
		NavContainer navCon;
		SensorsGui sensorsGui;
};

#endif

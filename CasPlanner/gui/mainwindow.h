#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <boost/signal.hpp>
#include <boost/bind.hpp>
#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QMainWindow>
#include "robotmanager.h"
#include "statusbar.h"
#include "tabcontainer.h"
class MainWindow : public QMainWindow 
{
Q_OBJECT
public:
	MainWindow(QWidget *parent=0); 
	MainWindow(QStringList strings, QWidget *parent=0); 
	~MainWindow(); 
    public slots:
		void logData();
		void commStart(); 
private: 
    TabContainer * tabcontainer;
    RobotManager * robotManager;
	StatusLogger * statusLogger; 
	int logCount; 
};
#endif


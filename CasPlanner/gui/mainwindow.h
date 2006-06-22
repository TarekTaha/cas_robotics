#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include "robotcomm.h"
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
		void loadRobot(); 
private: 
    TabContainer tabcontainer;
    RobotComm * robotcomm;
	//QTMapDataInterface mapManager;
	int logCount; 
	StatusLogger *statusLogger; 
};
#endif


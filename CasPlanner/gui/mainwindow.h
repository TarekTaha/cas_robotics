#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <QMainWindow>
#include "statusbar.h"
#include "cfgreader.h"
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
	//QTMapDataInterface mapManager;
	int logCount; 
	StatusLogger *statusLogger; 
	CfgReader *cf; 
};
#endif


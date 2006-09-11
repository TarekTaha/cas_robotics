#include "mainwindow.h"
#include <QPushButton>
#include<QVBoxLayout>
#include<QTimer>
#include <unistd.h>
MainWindow::MainWindow(QWidget *parent):
QMainWindow(parent)
{
    statusLogger = new StatusLogger(statusBar()); 
}

MainWindow::MainWindow(QStringList configFiles, QWidget *parent):
    QMainWindow(parent), logCount(0)
{
    // Create the PlayGround where all the RobotManagers will run
    playGround = new PlayGround (configFiles);
    QWidget *container = new QWidget(this); 
    tabcontainer = new TabContainer(parent,playGround);
//    robotManager->setNavContainer(tabcontainer->navCon);
    playGround->setNavContainer(tabcontainer->navCon);
    QVBoxLayout *vLayout = new QVBoxLayout;
    QPushButton *emergStop = new QPushButton(" STOP ROBOT ");  
    QPushButton *connRobot = new QPushButton(" Connect to Robot ");
    QPushButton *logButton = new QPushButton(" Show Loggs");    
    QPalette palette = emergStop->palette();
    palette.setColor(QPalette::Button, Qt::red);
    emergStop->setPalette(palette); 
    palette.setColor(QPalette::Button, Qt::green);
    connRobot->setPalette(palette);
    palette.setColor(QPalette::Button, Qt::yellow);
    logButton->setPalette(palette);    
    setMinimumSize(QSize(900,700)); 
    QHBoxLayout *layout = new QHBoxLayout, *layout2 = new QHBoxLayout; 
    layout->addWidget(tabcontainer,1); 
    layout2->addWidget(logButton);
    layout2->addWidget(emergStop);
    layout2->addWidget(connRobot);
    vLayout->addLayout(layout); 
    vLayout->addLayout(layout2);     
    container->setLayout(vLayout);
    
    statusBar()->showMessage("Welcome to CAS Navigation System ...", 20000); 
    statusBar()->showMessage("Initialization Done.");

    statusLogger = new StatusLogger(statusBar()); 
    setCentralWidget(container);
    statusLogger->addStatusMsg(0,1,"Start of Loggs ..."); 

//    connect(playGround->commManager, SIGNAL(statusMsg(int,int,QString)), statusLogger, SLOT(addStatusMsg(int,int,QString))); 
    connect(emergStop, SIGNAL(pressed()), playGround, SLOT(stopRobots()));
    connect(connRobot, SIGNAL(pressed()), playGround, SLOT(startRobotsComm()));
    connect(logButton, SIGNAL(clicked()),statusLogger, SLOT(showLog())); 
    statusLogger->addStatusMsg(0,1,"Navigation System Started ... "); 
	// Data Logging Timer
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(logData()));
    timer->start(60000);
}
void MainWindow::logData()
{
	QApplication::beep();
    //qDebug("Am not sure , but i might be Logging :)"); 
    return;
}

void MainWindow::commStart()
{
}

MainWindow::~MainWindow()
{

}


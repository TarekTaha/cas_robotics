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
    // Reads Robot Configurations from the file(s)
    robotManager = new RobotManager(configFiles);
    QWidget *container = new QWidget(this); 
    tabcontainer = new TabContainer(parent,robotManager);
    robotManager->setNavContainer(tabcontainer->navCon);
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

    //Comms is now set up, connect map view to map manager. 
    qDebug("Initializing Tabs"); 
    connect(robotManager->commManager, SIGNAL(statusMsg(int,int,QString)), statusLogger, SLOT(addStatusMsg(int,int,QString))); 
//    connect(emergStop, SIGNAL(pressed()), robotManager->commManager, SLOT(emergencyStop()));
    connect(emergStop, SIGNAL(pressed()), robotManager->navigator, SLOT(StopNavigating()));
    connect(connRobot, SIGNAL(pressed()), this, SLOT(commStart()));
    connect(logButton, SIGNAL(clicked()),statusLogger, SLOT(showLog())); 
    statusLogger->addStatusMsg(0,1,"GUI started successfully ... "); 

    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(logData()));
    timer->start(60000);
}
void MainWindow::logData()
{
    //qDebug("Am not sure , but i might be Logging :)"); 
    return;
}

void MainWindow::commStart()
{
    robotManager->startComms();
}

MainWindow::~MainWindow()
{

}


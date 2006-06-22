#include "mainwindow.h"
#include <QPushButton>
#include<QVBoxLayout>
#include<QTimer>
#include <unistd.h>
// No configuration file given, so do the defualt configs
MainWindow::MainWindow(QWidget *parent):
QMainWindow(parent)
{
    //missionView.setMapManager(&mapManager); 
    statusLogger = new StatusLogger(statusBar()); 
    //setCentralWidget(&missionView); 
}

MainWindow::MainWindow(QStringList configFiles, QWidget *parent):
    QMainWindow(parent), logCount(0)
{
    QWidget *container = new QWidget(this); 
    QVBoxLayout *vLayout = new QVBoxLayout;
    QPushButton *emergStop = new QPushButton(" STOP ROBOT ");  
    QPalette palette = emergStop->palette();
    palette.setColor(QPalette::Button, Qt::yellow);
    emergStop->setPalette(palette); 
    setMinimumSize(QSize(900,700)); 
    QHBoxLayout *layout = new QHBoxLayout; 
    layout->addWidget(&tabcontainer,1); 
    vLayout->addLayout(layout); 
    vLayout->addWidget(emergStop); 
    container->setLayout(vLayout);
    statusBar()->showMessage("Welcome to CAS Navigation System ...", 20000); 
    statusBar()->showMessage("Initialization Done.");
    QPushButton *loadRobotBtn = new QPushButton("EXIT"); 
    statusBar()->addPermanentWidget(loadRobotBtn);
    connect(loadRobotBtn, SIGNAL(clicked()), this, SLOT(loadRobot()));
    statusLogger = new StatusLogger(statusBar()); 
    setCentralWidget(container);
    // Reads Robot Configurations from the file(s)
    robotcomm = new RobotComm(configFiles);
    robotcomm->start();
    statusLogger->addStatusMsg(0,1,"Just a msg ..."); 
    //connect(commsmgr, SIGNAL(statusMsg(int,int,QString)), statusLogger, SLOT(addStatusMsg(int,int,QString))); 
    //Comms is now set up, connect map view to map manager. 
    qDebug("Initializing Tabs"); 
    //tabcontainer.setMapManager(&mapManager); 
    //To:Do
    //connect(emergStop, SIGNAL(pressed()), commsmgr, SLOT(emergencyStop())); 
    //cf = new CfgReader(configFiles, &robotsView, commsmgr);
    statusLogger->addStatusMsg(0,1,"Stating comms ... done"); 
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(logData()));
    timer->start(60000);
}
void MainWindow::logData()
{
    //logCount++; 
    qDebug("!!!!! Logging!!!!!!!"); 
    QString prefix = QString("log_%1_").arg(logCount); 
    //mapManager.writeOut(prefix, true); 
}

void MainWindow::loadRobot()
{
    //QString fileName = QFileDialog::getOpenFileName(this, "Load Robot ... "); 
    //cf->readRobot(fileName); 
}

MainWindow::~MainWindow()
{

}


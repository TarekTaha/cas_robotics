#include "mainwindow.h"
#include <QPushButton>
#include<QVBoxLayout>
#include<QTimer>
#include <unistd.h>
MainWindow::MainWindow(QWidget *parent):
QMainWindow(parent)
{
    statusLogger = new StatusLogger(statusBar()); 
    //setCentralWidget(&missionView); 
}

MainWindow::MainWindow(QStringList configFiles, QWidget *parent):
    QMainWindow(parent), logCount(0)
{
    // Reads Robot Configurations from the file(s)
    robotManager = new RobotManager(configFiles);

    QWidget *container = new QWidget(this); 
    tabcontainer = new TabContainer(this,robotManager);
    QVBoxLayout *vLayout = new QVBoxLayout;
    QPushButton *emergStop = new QPushButton(" STOP ROBOT ");  
    QPalette palette = emergStop->palette();
    palette.setColor(QPalette::Button, Qt::yellow);
    emergStop->setPalette(palette); 
    setMinimumSize(QSize(900,700)); 
    QHBoxLayout *layout = new QHBoxLayout; 
    layout->addWidget(tabcontainer,1); 
    vLayout->addLayout(layout); 
    vLayout->addWidget(emergStop); 
    container->setLayout(vLayout);
    statusBar()->showMessage("Welcome to CAS Navigation System ...", 20000); 
    statusBar()->showMessage("Initialization Done.");
    QPushButton *commStart = new QPushButton("Connect"); 
    statusBar()->addPermanentWidget(commStart);
    connect(commStart, SIGNAL(clicked()), this, SLOT(commStart()));
    statusLogger = new StatusLogger(statusBar()); 
    setCentralWidget(container);
    statusLogger->addStatusMsg(0,1,"Testing Logging ..."); 
    connect(robotManager, SIGNAL(statusMsg(int,int,QString)), statusLogger, SLOT(addStatusMsg(int,int,QString))); 
    //Comms is now set up, connect map view to map manager. 
    qDebug("Initializing Tabs"); 
    connect(emergStop, SIGNAL(pressed()), robotManager, SLOT(emergencyStop())); 
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

void MainWindow::commStart()
{
    robotManager->startComms();
}

MainWindow::~MainWindow()
{

}


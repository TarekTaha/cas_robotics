#include "missioncontroltab.h"
#include "ui_missioncontroltab.h"

MissionControlTab::MissionControlTab(QWidget *parent,PlayGround *playGround_in) :
    QWidget(parent),
    ui(new Ui::MissionControlTab),
    playGround(playGround_in),
    robotInitialization(true),
    path(NULL)
{
    ui->setupUi(this);
    mapViewer = new MapViewer(this,playGround);
    ui->vLayout->addWidget(mapViewer);
    connect(mapViewer, SIGNAL(setStart(Pose)),  this, SLOT(setStart(Pose)));
    connect(mapViewer, SIGNAL(setEnd(Pose))  ,  this, SLOT(setEnd(Pose)));
    playGround->setMapViewer(mapViewer);
    if(playGround)
    {
        if(playGround->robotPlatforms.size()>0)
            playGround->activeRobot = playGround->robotPlatforms[0];
    }
    RobotManager *temp= NULL;
    QRadioButton *rob;
    if(playGround)
    {
        for(int i=0; i < playGround->robotPlatforms.size(); i++)
        {
            temp = playGround->robotPlatforms[i];
            rob = new QRadioButton(QString("Robot: ").append(temp->robot->robotName));
            connect( playGround->robotPlatforms[i]->planningManager,SIGNAL(pathFound(Node*)),this,SLOT(pathFound(Node*)));
            availableRobots.push_back(rob);
        }
        if(playGround->robotPlatforms.size()>0)
            playGround->activeRobot = playGround->robotPlatforms[0];
    }

    for(int i=0;i<availableRobots.size();i++)
    {
        ui->robotsL->addWidget(availableRobots[i]);
        connect(availableRobots[i],SIGNAL(toggled(bool )), this,SLOT(updateSelectedRobot(bool)));
    }
    connect(ui->pathPlanBtn,            SIGNAL(pressed()),this, SLOT(pathPlan()));
    connect(ui->captureImage,           SIGNAL(pressed()),this, SLOT(save()));
    connect(ui->pathFollowBtn,          SIGNAL(pressed()),this, SLOT(pathFollow()));
    connect(ui->pauseBtn,               SIGNAL(pressed()),this, SLOT(setNavigation()));
    connect(ui->intentionRecognitionBtn,SIGNAL(pressed()),this, SLOT(startIntentionRecognition()));
    connect(ui->resetBeliefBtn, 	SIGNAL(pressed()),this, SLOT(resetDestinationBelief()));
    if(availableRobots.size()>0)
        availableRobots[0]->setChecked(true);
}

MissionControlTab::~MissionControlTab()
{
    delete ui;
}


void MissionControlTab::resetDestinationBelief()
{
    if(this->playGround->activeRobot)
    {
        if(!playGround->activeRobot->intentionRecognizer)
        {
            QMessageBox msgBox(QMessageBox::Warning,QString("Warning"),QString("Intention Recognition not started, you want to start it now?"),QMessageBox::Ok|QMessageBox::Cancel,this,Qt::Dialog);
            msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
            switch (msgBox.exec())
            {
            case QMessageBox::Yes:
                startIntentionRecognition();
                break;
            case QMessageBox::No:
                return;
                break;
            default:
                return;
                break;
            }
        }
        playGround->activeRobot->intentionRecognizer->resetBelief();
    }
    else
    {
        qDebug()<<"Select a Robot and Start the intention Recognizer First !!!";
    }
}

void MissionControlTab::startIntentionRecognition()
{
    if(this->playGround->activeRobot)
    {
        if(!playGround->activeRobot->commManager->isConnected())
        {
            QMessageBox msgBox(QMessageBox::Warning,QString("Warning"),QString("Your not Connected to the Robot, do you want me to connect?"),QMessageBox::Ok|QMessageBox::Cancel,this,Qt::Dialog);
            msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
            switch (msgBox.exec())
            {
            case QMessageBox::Yes:
                playGround->activeRobot->startComms();
                break;
            case QMessageBox::No:
                return;
                break;
            default:
                return;
                break;
            }
        }
        playGround->activeRobot->startIntentionRecognizer();
    }
    else
    {
        qDebug()<<"No Robot Selected";
    }
}

void MissionControlTab::pathTraversed()
{
    if(playGround->activeRobot->navigator->isRunning())
    {
        playGround->activeRobot->navigator->StopNavigating();
        playGround->activeRobot->navigator->quit();
        qDebug()<<"Quitting Thread";
    }
    ui->pathFollowBtn->setText("Path Follow");
    ui->pauseBtn->setText("Pause");
    playGround->activeRobot->notFollowing = true;
}

void MissionControlTab::pathFollow()
{
    if(!playGround->activeRobot->commManager)
    {
        qDebug()<<"\t NavTab: Communication Manager Not Initialized";
        return;
    }
    if(!playGround->activeRobot->commManager->isConnected())
    {
        QMessageBox msgBox(QMessageBox::Warning,QString("Warning"),QString("Your not Connected to the Robot, do you want me to connect?"),QMessageBox::Ok|QMessageBox::Cancel,this,Qt::Dialog);
        msgBox.setStandardButtons(QMessageBox::Yes | QMessageBox::No);
        switch (msgBox.exec())
        {
        case QMessageBox::Yes:
            playGround->activeRobot->startComms();
            break;
        case QMessageBox::No:
            return;
            break;
        default:
            return;
            break;
        }
    }
    if(playGround->activeRobot->notFollowing)
    {
        if(playGround->activeRobot->navigator->isRunning())
        {
            playGround->activeRobot->navigator->quit();
        }
        if(path)
        {
            playGround->activeRobot->navigator->setPath(path);
            playGround->activeRobot->navigator->start();
            ui->pathFollowBtn->setText("Stop");
            playGround->activeRobot->notFollowing = false;
        }
    }
    else
    {
        if(playGround->activeRobot->navigator->isRunning())
        {
            playGround->activeRobot->navigator->StopNavigating();
            playGround->activeRobot->navigator->quit();
            qDebug()<<"Quitting Thread";
        }
        ui->pathFollowBtn->setText("Path Follow");
        playGround->activeRobot->notFollowing = true;
    }
}

void MissionControlTab::updateSelectedRobot(bool)
{
    for(int i=0;i<availableRobots.size();i++)
    {
        if(availableRobots[i]->isChecked())
        {
            playGround->activeRobot = playGround->robotPlatforms[i];
            break;
        }
    }
}

void MissionControlTab::setNavigation()
{
    if(playGround->activeRobot->notPaused)
    {
        playGround->activeRobot->navigator->setPause(true);
        playGround->activeRobot->notPaused = false;
        ui->pauseBtn->setText("Continue");
    }
    else
    {
        playGround->activeRobot->navigator->setPause(false);
        playGround->activeRobot->notPaused = true;
        ui->pauseBtn->setText("Pause");
    }
}

void MissionControlTab::pathFound(Node*)
{
    ui->pathPlanBtn->setEnabled(true);
}

void MissionControlTab::pathPlan()
{
    playGround->activeRobot->planningManager->setMap(playGround->mapManager->globalMap);
    playGround->activeRobot->planningManager->findPath(METRIC);
    ui->pathPlanBtn->setEnabled(false);
}

void MissionControlTab::generateSpace()
{
    playGround->activeRobot->planningManager->generateSpace();
}

void MissionControlTab::loadMap()
{

}

void MissionControlTab::save()
{
    mapViewer->saveImage();
}

void MissionControlTab::setStart(Pose startLoc)
{
    if(this->playGround->activeRobot)
    {
        playGround->activeRobot->planningManager->setStart(startLoc);
    }
    else
    {
        qDebug()<<"No Robot is Selected";
    }
}

void MissionControlTab::setEnd(Pose endLoc)
{
    if(this->playGround->activeRobot)
    {
        playGround->activeRobot->planningManager->setEnd(endLoc);
    }
    else
    {
        qDebug()<<"No Robot is Selected";
    }
}

void MissionControlTab::setMap(Map * map)
{
    if(this->playGround->activeRobot)
    {
        playGround->activeRobot->planningManager->setMap(map);
    }
    else
    {
        qDebug()<<"No Robot is Selected";
    }
}

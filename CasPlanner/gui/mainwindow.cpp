/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "mainwindow.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <QTimer>
#include <unistd.h>
#include "logger.h"
#include "playground.h"
#include "statusbar.h"
#include "tabcontainer.h"
#include "settings.h"

MainWindow::MainWindow(QWidget *parent):
QMainWindow(parent)
{

}

MainWindow::MainWindow(QStringList configFiles, QWidget *parent):
    QMainWindow(parent), logCount(0)
{
    Logger& logger = Logger::getLogger();
    logger.init("log.txt",true);
    logger.setLevel(Logger::Debug);
    // Create the PlayGround where all the RobotManagers will run
    playGround = new PlayGround(configFiles,statusBar());

    QWidget *container = new QWidget(this);
    tabcontainer = new TabContainer(parent,playGround);

    // Buttons
    QVBoxLayout *vLayout 		 = new QVBoxLayout;
    QPushButton *emergStop 		 = new QPushButton(" STOP ROBOT ");
    QPushButton *logButton 		 = new QPushButton(" Show Loggs");
    QPalette palette = emergStop->palette();
    palette.setColor(QPalette::Button, Qt::red);
    emergStop->setPalette(palette);
    palette.setColor(QPalette::Button, Qt::green);
    palette.setColor(QPalette::Button, Qt::yellow);
    logButton->setPalette(palette);

    // Window Layout
    setMinimumSize(QSize(900,730));
    QHBoxLayout *layout = new QHBoxLayout, *layout2 = new QHBoxLayout;
    layout->addWidget(tabcontainer,1);
    layout2->addWidget(logButton);
    layout2->addWidget(emergStop);
    vLayout->addLayout(layout);
    vLayout->addLayout(layout2);
    container->setLayout(vLayout);

    setCentralWidget(container);

    connect(emergStop, SIGNAL(pressed()), playGround, SLOT(stopRobots()));
    connect(logButton, SIGNAL(clicked()), playGround->statusLogger, SLOT(showLog()));


    statusBar()->showMessage("Welcome to CAS Navigation System ...", 20000);
    statusBar()->showMessage("Initialization Done.");
    imageCounter = 0;
}

void MainWindow::captureScreenShot()
{
    char counter[8];
    sprintf(counter,"%06d",imageCounter++);
    originalPixmap = QPixmap::grabWindow(QMainWindow::centralWidget()->winId());
    fileName = QString("./logs/screenshots/shot%1").arg(counter).append(".jpeg");
    originalPixmap.save(fileName,"JPEG",50);
}

void MainWindow::commStart()
{
}

MainWindow::~MainWindow()
{

}


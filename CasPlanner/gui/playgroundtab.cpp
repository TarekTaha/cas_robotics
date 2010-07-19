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
#include "playgroundtab.h"

Interfaces::Interfaces(QWidget *parent)
:QWidget(parent),
vLayout(NULL)
{
	setMinimumSize(200,200);
	vLayout = new QVBoxLayout;
	setLayout(vLayout);
}

void Interfaces::addInterface(DeviceType dev,QString name)
{
    QCheckBox *device = new QCheckBox(name);
    if(dev.isSubscribed())
    {
        device->setCheckState(Qt::Checked);
    }
    else
        device->setCheckState(Qt::Unchecked);
    vLayout->addWidget(device);
    devicesBox.push_back(device);
    chk2Dev.insert(device,dev);
    connect(device,SIGNAL(stateChanged(int)),this,SLOT(checkChanged(int)));
    return;
}

void Interfaces::checkChanged(int state)
{
        qDebug("State is %d",state);
}

void Interfaces::createIcons(QVector <DeviceType> * devices)
{
    if(!devices)
        return;
    if(vLayout)
    {
        for(int i=0;i< devicesBox.size();i++)
            delete devicesBox[i];
    }
    update();
    //char section[256];
    this->devicesBox.clear();
    this->chk2Dev.clear();
    for (int i = 0; i < devices->size(); i++)
    {
        //snprintf(section, sizeof(section), "%s:%d ",LookupName((*devices)[i].addr.interf), (*devices)[i].addr.index);

        //printf("%-16s %-40s", section, qPrintable((*devices)[i].driverName));
        //if(!(*devices)[i].subscribed)
        addInterface((*devices)[i],QString((*devices)[i].getInterfaceName()).append((*devices)[i].getDriverName()));
        //printf("\n");
    }
}

PlayGroundTab::~PlayGroundTab()
{
}

PlayGroundTab::PlayGroundTab(QWidget * parent,PlayGround *playG):
	QWidget(parent),
	playGround(playG)
{
    contentsWidget = new QListWidget;
    contentsWidget->setViewMode(QListView::IconMode);
    contentsWidget->setIconSize(QSize(180, 84));
    contentsWidget->setMovement(QListView::Static);
    contentsWidget->setMaximumWidth(180);
    contentsWidget->setMaximumHeight(800);
    contentsWidget->setSpacing(12);

    pagesWidget = new QStackedWidget;
    pagesWidget->addWidget(new RobotConfigPage(this,playGround));
    pagesWidget->addWidget(new MapConfigPage(this,playGround));
    pagesWidget->addWidget(new ProfileConfigPage(this,playGround));

    createIcons();
    contentsWidget->setCurrentRow(0);

    QHBoxLayout *horizontalLayout = new QHBoxLayout;
    horizontalLayout->addWidget(contentsWidget);
    horizontalLayout->addWidget(pagesWidget, 1);

    QHBoxLayout *buttonsLayout = new QHBoxLayout;
    buttonsLayout->addStretch(1);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addLayout(horizontalLayout);
    //mainLayout->addStretch(1);
    mainLayout->addSpacing(12);
    mainLayout->addLayout(buttonsLayout);
    setLayout(mainLayout);

    setWindowTitle("Config Dialog");
}

void PlayGroundTab::createIcons()
{
    QListWidgetItem *configButton = new QListWidgetItem(contentsWidget);
    configButton->setIcon(QIcon(":/robot.jpg"));
    configButton->setText(("Robots Configuration"));
    configButton->setTextAlignment(Qt::AlignHCenter);
    configButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    QListWidgetItem *updateButton = new QListWidgetItem(contentsWidget);
    updateButton->setIcon(QIcon(":/lab.png"));
    updateButton->setText(("Map Configurations"));
    updateButton->setTextAlignment(Qt::AlignHCenter);
    updateButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    QListWidgetItem *queryButton = new QListWidgetItem(contentsWidget);
    queryButton->setIcon(QIcon(":/flagein.jpg"));
    queryButton->setText(("Profile Configuration"));
    queryButton->setTextAlignment(Qt::AlignHCenter);
    queryButton->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);

    connect(contentsWidget,SIGNAL(currentItemChanged(QListWidgetItem *, QListWidgetItem *)),
            (PlayGroundTab*)this, SLOT(changePage(QListWidgetItem *, QListWidgetItem*)));
}

void PlayGroundTab::changePage(QListWidgetItem *current, QListWidgetItem *previous)
{
    if (!current)
                current = previous;
    pagesWidget->setCurrentIndex(contentsWidget->row(current));
}

RobotConfigPage::RobotConfigPage(QWidget * parent,PlayGround *playG):
	QWidget(parent),
	playGround(playG),
	robotName("Robot's  Name: "),
	robotIp("Ip Address: "),
	robotPort("Port: "),
	robotLength("Robot's Length: "),
	robotWidth("Robot's Width: "),
	robotModel("Motion Model: "),
	robotCenter("Center of Rotation: "),
	robotMass("Robot's   Mass: "),
	robotInirtia("Robot's Inirita: "),
	modelDiff("Differential"),
	modelCar("Car Like"),
	laserInterfaceBox("Laser Interface"),
	posInterfaceBox("Position2d Interface"),
	vfhInterfaceBox("VFH Interface"),
	LocalizerInterfaceBox("Localizer Interface")
{
    QGroupBox *robotsGroup = new QGroupBox(tr("Available Robots"));
    QGroupBox *configGroup = new QGroupBox(tr("Robot Settings"));
    QGroupBox *interfaceGroup = new QGroupBox(tr("Supported Interfaces"));

    robotsCombo = new QComboBox;
    if(!playGround)
        return;
    for(int i=0;i<playGround->robotPlatforms.size();i++)
    {
        robotsCombo->addItem(playGround->robotPlatforms[i]->robot->robotName);
    }

    QHBoxLayout *robotsHLayout = new QHBoxLayout;
    robotsHLayout->addWidget(&robotName);
    robotsHLayout->addWidget(robotsCombo);

    QVBoxLayout *robotsVLayout = new QVBoxLayout;
    robotsVLayout->addLayout(robotsHLayout);
    robotsGroup->setLayout(robotsVLayout);

    QHBoxLayout *configHLayout = new QHBoxLayout;
    configHLayout->addWidget(&robotName);
    configHLayout->addWidget(&robotNameE);

    configHLayout->addWidget(&robotIp);
    configHLayout->addWidget(&robotIpE);
    configHLayout->addWidget(&robotPort);
    configHLayout->addWidget(&robotPortE);
    robotPortE.setMinimum(0);
    robotPortE.setMaximum(7000);
    robotPortE.setSingleStep(1);
    robotPortE.setValue(6665);
    robotPortE.setDecimals(0);

    QHBoxLayout *configH2Layout = new QHBoxLayout;
    configH2Layout->addWidget(&robotLength);
    configH2Layout->addWidget(&robotLengthE);
    robotLengthE.setMinimum(-10);
    robotLengthE.setMaximum(10);
    robotLengthE.setSingleStep(0.01);
    robotLengthE.setValue(0);

    configH2Layout->addWidget(&robotWidth);
    configH2Layout->addWidget(&robotWidthE);
    robotWidthE.setMinimum(-10);
    robotWidthE.setMaximum(10);
    robotWidthE.setSingleStep(0.01);
    robotWidthE.setValue(0);

    QHBoxLayout *configH3Layout = new QHBoxLayout;
    configH3Layout->addWidget(&robotMass);
    configH3Layout->addWidget(&robotMassE);
    robotMassE.setMinimum(0);
    robotMassE.setMaximum(1000);
    robotMassE.setSingleStep(0.5);
    robotMassE.setValue(0);

    configH3Layout->addWidget(&robotInirtia);
    configH3Layout->addWidget(&robotInirtiaE);
    robotInirtiaE.setMinimum(0);
    robotInirtiaE.setMaximum(1000);
    robotInirtiaE.setSingleStep(0.5);
    robotInirtiaE.setValue(0);

    QHBoxLayout *configH4Layout = new QHBoxLayout;
    configH4Layout->addWidget(&robotModel);

    QVBoxLayout *showL = new QVBoxLayout;
    showL->addWidget(&modelDiff);
    showL->addWidget(&modelCar);
    modelDiff.setChecked(true);

    configH4Layout->addLayout(showL);
    configH4Layout->addWidget(&robotCenter);
    configH4Layout->addWidget(&robotCenterX);
    configH4Layout->addWidget(&robotCenterY);
    robotCenterX.setMinimum(-5);
    robotCenterX.setMaximum(5);
    robotCenterX.setSingleStep(0.01);
    robotCenterX.setValue(0);
    robotCenterY.setMinimum(-5);
    robotCenterY.setMaximum(5);
    robotCenterY.setSingleStep(0.01);
    robotCenterY.setValue(0);

    QVBoxLayout *configVLayout = new QVBoxLayout;
    configVLayout->addLayout(configHLayout);
    configVLayout->addLayout(configH2Layout);
    configVLayout->addLayout(configH3Layout);
    configVLayout->addLayout(configH4Layout);
    configGroup->setLayout(configVLayout);

    //	QLabel * L = new QLabel("Connected Interfaces");
    //	QLabel * A = new QLabel("Available Interfaces");
    //    QHBoxLayout *interfaceHLayout = new QHBoxLayout;
    //    interfaceHLayout->addWidget(L);
    //    interfaceHLayout->addWidget(A);
    //
    //    QHBoxLayout *interfaceH2Layout = new QHBoxLayout;
    //	//robotInterfaces = new RobotInterfaces(this);
    //	interfacesList  = new InterfacesList(this);
    ////    interfaceH2Layout->addWidget(robotInterfaces);
    //    interfaceH2Layout->addWidget(interfacesList);

    QHBoxLayout *interfaceH2Layout = new QHBoxLayout;
    interfaces = new Interfaces(this);
    interfaceH2Layout->addWidget(interfaces);
    //
    QVBoxLayout *interfaceVLayout = new QVBoxLayout;
    //    interfaceVLayout->addLayout(interfaceHLayout);
    interfaceVLayout->addLayout(interfaceH2Layout);
    interfaceGroup->setLayout(interfaceVLayout);


    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(robotsGroup);
    mainLayout->addWidget(configGroup);
    mainLayout->addWidget(interfaceGroup);
    mainLayout->addStretch(1);
    setLayout(mainLayout);
    connect(robotsCombo,SIGNAL(highlighted(int)),this,SLOT(updateSelection(int)));
    updateSelection(0);
}
void RobotConfigPage::updateSelection(int r)
{
    if(!playGround)
        return;
    if(r > (playGround->robotPlatforms.size()-1))
        return;
    robotNameE.setText(playGround->robotPlatforms[r]->robot->robotName);
    robotIpE.setText(playGround->robotPlatforms[r]->robot->robotIp);
    robotCenterX.setValue(playGround->robotPlatforms[r]->robot->robotCenter.x());
    robotCenterY.setValue(playGround->robotPlatforms[r]->robot->robotCenter.y());
    robotPortE.setValue(playGround->robotPlatforms[r]->robot->robotPort);
    robotLengthE.setValue(playGround->robotPlatforms[r]->robot->robotLength);
    robotWidthE.setValue(playGround->robotPlatforms[r]->robot->robotWidth);
    robotMassE.setValue(playGround->robotPlatforms[r]->robot->robotMass);
    robotInirtiaE.setValue(playGround->robotPlatforms[r]->robot->robotMI);
    if(playGround->robotPlatforms[r]->robot->robotModel == "diff")
        modelDiff.setChecked(true);
    else
        modelCar.setChecked(true);
    //	interfacesList->clear();
    //	robotInterfaces->clear();
    if(!playGround)
    {
        qDebug("playGround NULL"); fflush(stdout);
        return;
    }
    if(!playGround->robotPlatforms[r])
    {
        qDebug("robotPlatform NULL"); fflush(stdout);
        return;
    }
    if(!playGround->robotPlatforms[r]->commManager)
    {
        qDebug("commManager NULL"); fflush(stdout);
        return;
    }
    //	QVector <DeviceType> * d = playGround->robotPlatforms[r]->commManager->getDevices();
    //	if(d)
    //		interfaces->createIcons(d);
    //	robotInterfaces->createIcons(d);
    //	interfacesList->createIcons(d);
}

MapConfigPage::MapConfigPage(QWidget * parent,PlayGround *playG):
	QWidget(parent),
	playGround(playG),
	browseMapBtn("Browse"),
	reloadMapBtn("Reload Map"),
	whiteFree("White is Free Space"),
	blackFree("Black is Free Space")
{
    mapGround = new QGroupBox(tr("Map Configurations"));
    mapName = new QLabel(tr("MapName:"));
    mapNameEdit = new QLineEdit;

    mapRes = new QLabel(tr("Resolution:"));;
    mapResolution.setMinimum(0.01);
    mapResolution.setMaximum(2);
    mapResolution.setSingleStep(0.01);

    if(playGround->mapManager->globalMap)
    {
        fileName = playGround->mapManager->mapName;
        mapNameEdit->setText(playGround->mapManager->mapName);
        mapResolution.setValue(playGround->mapManager->globalMap->mapRes);
        if(playGround->mapManager->mapNegate)
            blackFree.setChecked(true);
        else
            whiteFree.setChecked(true);
    }
    else
    {
        mapResolution.setValue(0);
    }

    QGridLayout *mapConfigLayout = new QGridLayout;
    mapConfigLayout->addWidget(mapName, 0, 0);
    mapConfigLayout->addWidget(mapNameEdit, 0, 1);
    mapConfigLayout->addWidget(&browseMapBtn, 0, 2);
    mapConfigLayout->addWidget(mapRes, 1, 0);
    mapConfigLayout->addWidget(&mapResolution, 1, 1);
    mapConfigLayout->addWidget(&whiteFree, 2, 0);
    mapConfigLayout->addWidget(&blackFree, 2, 1);
    mapConfigLayout->addWidget(&reloadMapBtn,3, 0);

    connect(&browseMapBtn,SIGNAL(pressed()),this,SLOT(getFileName()));
    connect(&reloadMapBtn,SIGNAL(pressed()),this,SLOT(reloadMap()));

    mapGround->setLayout(mapConfigLayout);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(mapGround);
    mainLayout->addSpacing(12);
    mainLayout->addStretch(1);
    setLayout(mainLayout);
}

void MapConfigPage::getFileName()
{
    fileName = QFileDialog::getOpenFileName(this, tr("Open Map File"),
                                            "./resources",
                                            tr("Images (*.png *.xpm *.jpg *.bmp *.jpeg)"));
    mapNameEdit->setText(fileName);
}

void MapConfigPage::reloadMap()
{
    if((playGround->mapManager->mapName != fileName) || (playGround->mapManager->globalMap->mapRes  != float(mapResolution.value())))
    {
        bool negate;
        if(whiteFree.isChecked())
            negate = false;
        else
            negate = true;
        playGround->mapManager->mapName = fileName;
        playGround->loadMap(fileName,mapResolution.value(),negate,Pose(0,0,0));
        mapNameEdit->setText(playGround->mapManager->mapName);
    }
}

ProfileConfigPage::ProfileConfigPage(QWidget * parent,PlayGround *playG):
	QWidget(parent),
	playGround(playG)
{
    QGroupBox *profileGroup = new QGroupBox(tr("User Profile"));

    QLabel *nameLabel = new QLabel(tr("User Name:"));
    QLineEdit *nameEdit = new QLineEdit;

    QGridLayout *packagesLayout = new QGridLayout;
    packagesLayout->addWidget(nameLabel, 0, 0);
    packagesLayout->addWidget(nameEdit, 0, 1);
    profileGroup->setLayout(packagesLayout);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(profileGroup);
    mainLayout->addSpacing(12);
    mainLayout->addStretch(1);
    setLayout(mainLayout);
}

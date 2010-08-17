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
#ifndef PLAYGROUNDTAB_H_
#define PLAYGROUNDTAB_H_

#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>

#include <QWidget>
#include <QVector>
#include <QHash>
#include <QLabel>
#include <QLineEdit>
#include <QRadioButton>
#include <QAction>
#include <QButtonGroup>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QListView>
#include <QSplitter>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QDialog>
#include <QFrame>
#include <QHash>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QPushButton>

#include "playerinterface.h"

class PlayGround;
class PlayGroundTab;
class PlanningTab;
class QListWidget;
class QListWidgetItem;
class QStackedWidget;
class QComboBox;

class Interfaces : public QWidget
{
    Q_OBJECT
public:
    Interfaces(QWidget *parent = 0);
    void createIcons(QVector <DeviceType> * devices);
    void addInterface(DeviceType dev,QString name);
public Q_SLOTS:
    void checkChanged(int state);
private:
    QVBoxLayout * vLayout;
    QVector <QCheckBox *> devicesBox;
    QHash   <QCheckBox *, DeviceType> chk2Dev;
};

class RobotConfigPage : public QWidget
{
    Q_OBJECT
public:
    RobotConfigPage(QWidget *parent = 0,PlayGround *playG=0);
public Q_SLOTS:
    void updateSelection(int r);
public:
    PlayGround * playGround;
    QComboBox  * robotsCombo;
    Interfaces * interfaces;
    QLabel robotName,robotIp,robotPort,robotLength,robotWidth,robotModel,robotCenter,robotMass,
    robotInirtia;
    QLineEdit robotNameE,robotIpE;
    QRadioButton modelDiff,modelCar;
    QDoubleSpinBox robotCenterX,robotCenterY,robotPortE,robotLengthE,robotWidthE,robotMassE,
    robotInirtiaE;
    QCheckBox laserInterfaceBox,posInterfaceBox,vfhInterfaceBox,LocalizerInterfaceBox;
};

class MapConfigPage : public QWidget
{
    Q_OBJECT
public:
    MapConfigPage(QWidget *parent = 0,PlayGround *playG=0);
    PlayGround * playGround;
    QGroupBox *mapGround;
    QString fileName;
    QLabel *mapName,*mapRes ;
    QLineEdit *mapNameEdit ;
    QDoubleSpinBox mapResolution;
    QPushButton browseMapBtn,reloadMapBtn;
    QRadioButton whiteFree,blackFree;
public Q_SLOTS:
    void getFileName();
    void reloadMap();
};

class ProfileConfigPage : public QWidget
{
    Q_OBJECT
public:
    ProfileConfigPage(QWidget *parent = 0,PlayGround *playG=0);
    PlayGround * playGround;
};

class PlayGroundTab : public  QWidget
{
    Q_OBJECT
public:
    virtual ~PlayGroundTab();
    PlayGroundTab(QWidget *parent=0,PlayGround *playG=0);
    PlayGround * playGround;
public Q_SLOTS:
    void changePage(QListWidgetItem *current, QListWidgetItem *previous);
private:
    void createIcons();
    QListWidget     *contentsWidget;
    QStackedWidget  *pagesWidget;
};

#endif /*PLAYGROUNDTAB_H_*/

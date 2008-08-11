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
#include <libplayercore/player.h>

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QListView>
#include <QtGui/QSplitter>
#include <QtGui/QTableWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include <QtGui/QDialog>
#include <QFrame>
#include <QHash>
#include "playground.h"

class PlayGround;
class QListWidget;
class QListWidgetItem;
class QStackedWidget;
class PlayGroundTab;
class QDragEnterEvent;
class QDropEvent;
class QMouseEvent;

class Interfaces : public QWidget
{
	Q_OBJECT
	public:
		Interfaces(QWidget *parent = 0);
		void createIcons(QVector <DeviceType> * devices);
		void addInterface(DeviceType dev,QString name);
	public slots:		
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
    public slots:
		void updateSelection(int r);
	public:    	
    	PlayGround * playGround;
    	QComboBox  * robotsCombo;
    	Interfaces * interfaces;
//    	InterfacesList * interfacesList;
//    	RobotInterfaces * robotInterfaces;
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
    public slots:    	
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
	public slots:
    	void changePage(QListWidgetItem *current, QListWidgetItem *previous);
	private:
    	void createIcons();
		QListWidget 	*contentsWidget;
    	QStackedWidget  *pagesWidget;
};

#endif /*PLAYGROUNDTAB_H_*/

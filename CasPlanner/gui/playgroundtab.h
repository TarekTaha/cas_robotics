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
 
//class InterfacesList : public QListWidget
//{
//	Q_OBJECT
//	public:
//    	InterfacesList(QWidget *parent = 0);
//     	void addInterface(DeviceType dev,QPixmap icon,QString name, QPoint location);
//		void createIcons(QVector <DeviceType> * devices);     	
//	public slots:
//		void itemSelectionChanged();
//	protected:		
//    	void dragEnterEvent(QDragEnterEvent *event);
//     	void dragMoveEvent(QDragMoveEvent *event);
//     	void dropEvent(QDropEvent *event);
//     	void startDrag(Qt::DropActions supportedActions);
//     	QHash <QListWidgetItem *, DeviceType> wi2Dev;
//};
//
//class RobotInterfaces : public QListWidget
//{
//	Q_OBJECT
//	public:
//    	RobotInterfaces(QWidget *parent = 0);
//     	void addInterface(QPixmap icon,QString name, QPoint location);
//		void createIcons(QVector <DeviceType> * devices);
////	public slots:
////		void itemSelectionChanged();
////		void itemChanged( QListWidgetItem * item ); 		     	
//	protected:
//    	void dragEnterEvent(QDragEnterEvent *event);
//     	void dragMoveEvent(QDragMoveEvent *event);
//     	void dropEvent(QDropEvent *event);
//     	void startDrag(Qt::DropActions supportedActions);
//};

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

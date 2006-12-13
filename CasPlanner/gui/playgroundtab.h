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
#include "playground.h"

class PlayGround;
class QListWidget;
class QListWidgetItem;
class QStackedWidget;
class PlayGroundTab;
class QDragEnterEvent;
class QDropEvent;

class DragToWidget : public QFrame
{
	public:
    	DragToWidget(QWidget *parent=0);
	protected:
    	void dragEnterEvent(QDragEnterEvent *event);
     	void dropEvent(QDropEvent *event);
     	void mousePressEvent(QMouseEvent *event);
};

class DragFromWidget : public QFrame
{
	public:
    	DragFromWidget(QWidget *parent=0);
	protected:
    	//void dragEnterEvent(QDragEnterEvent *event);
     	void dropEvent(QDropEvent *event);
     	void mousePressEvent(QMouseEvent *event);
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

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
class QMouseEvent;
 
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
		void createIcons(QVector <device_t> devices);    	
	protected:
    	//void dragEnterEvent(QDragEnterEvent *event);
    	QVector <QLabel> icons;
     	void dropEvent(QDropEvent *event);
     	void mousePressEvent(QMouseEvent *event);    	
};

class InterfacesList : public QListWidget
{
	Q_OBJECT
	public:
    	InterfacesList(QWidget *parent = 0);
     	void addPiece(QPixmap pixmap, QPoint location);
	protected:
    	void dragEnterEvent(QDragEnterEvent *event);
     	void dragMoveEvent(QDragMoveEvent *event);
     	void dropEvent(QDropEvent *event);
     	void startDrag(Qt::DropActions supportedActions);
};

class RobotInterfaces : public QWidget
{
    Q_OBJECT
	public:
    	RobotInterfaces(QWidget *parent = 0);
    	void clear();
	signals:
    	void puzzleCompleted();
	protected:
    	void dragEnterEvent(QDragEnterEvent *event);
     	void dragLeaveEvent(QDragLeaveEvent *event);
     	void dragMoveEvent(QDragMoveEvent *event);
     	void dropEvent(QDropEvent *event);
     	void mousePressEvent(QMouseEvent *event);
     	void paintEvent(QPaintEvent *event);
	private:
    	int findPiece(const QRect &pieceRect) const;
     	const QRect targetSquare(const QPoint &position) const;
     	QList<QPixmap> piecePixmaps;
     	QList<QRect> pieceRects;
     	QList<QPoint> pieceLocations;
     	QRect highlightedRect;
     	int inPlace;
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
//    	DragFromWidget * dragFromWidget;
//    	DragToWidget   * dragToWidget;
    	InterfacesList * interfacesList;
    	RobotInterfaces * robotInterfaces;
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

#ifndef NAVIGATIONTAB_H
#define NAVIGATIONTAB_H

#include <QWidget>
#include <mapviewer.h>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTreeWidget>
#include <QPushButton>
#include <QHash>

class NavControlPanel: public QWidget 
{
Q_OBJECT
    public:  
	NavControlPanel(QWidget *parent = 0);
    public slots:
	void updateMap(); 
	void handleSelection(); 
	void updateSelectedObject(double);
	void updateSelectedObject(int); 
	void load(); 
	void save();
	void captureMap(); 
	void exportHtml(); 
	void setToRoot(); 
    signals:
	void propsChanged(); 
    private: 
	//void setActionValues(MapObject *mo);
	QGroupBox showHideGB; 
	QCheckBox showGrids; 
	QCheckBox showOGs;
	QCheckBox showRobots; 
	QCheckBox showLabels; 
	QCheckBox showPointclouds; 
	QCheckBox showPatchBorders;
	QTreeWidget selectedObject;
	QGroupBox transformGB;
	QSpinBox visSB; 
	QDoubleSpinBox xSB; 
	QDoubleSpinBox ySB;
	QDoubleSpinBox zSB; 
	QDoubleSpinBox rSB;
	QDoubleSpinBox pSB;
	QDoubleSpinBox yaSB; 
	QPushButton setToRootBtn; 
	//QTMapDataInterface *mapManager;
	QGroupBox actionGB;
	QPushButton captureMapBtn; 
	QPushButton loadBtn; 
	QPushButton saveAsBtn;
	QPushButton exportBtn;
	//QHash<QTreeWidgetItem*, MapObject *> wiToMo;
	//QHash<MapObject *, QTreeWidgetItem *> moToWi;   
	friend class NavContainer; 
	
	static const int RobotNodeType = QTreeWidgetItem::UserType+1; 
	static const int PatchNodeType = QTreeWidgetItem::UserType+2;
	static const int SnapNodeType = QTreeWidgetItem::UserType+3;
	
};

class NavContainer : public QWidget
{
Q_OBJECT
    public:
	NavContainer(QWidget *parent = 0);
	~NavContainer();
	//void setMapManager(QTMapDataInterface *mapManager);
    private:
	MapViewer mapViewer;
	NavControlPanel navControlPanel; 
	//QTMapDataInterface *mapManager; 
	friend class NavControlPanel; 
};

#endif

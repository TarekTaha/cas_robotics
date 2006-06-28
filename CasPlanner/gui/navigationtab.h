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
		void load(); 
		void save();
		void captureMap(); 
		void exportHtml(); 
		void setToRoot(); 
    signals:
		void propsChanged(); 
    private: 
		
		// Planning Steps
		QGroupBox planningGB;
		QCheckBox bridgeTest;
		QCheckBox connectNodes;
		QCheckBox regGrid;
		QCheckBox obstPenalty;
		QCheckBox expandObst;
		QCheckBox showTree;

		// Planning Parameters
		QGroupBox parametersGB;
		QDoubleSpinBox obstExpRadSB;
		QDoubleSpinBox bridgeTestResSB; 
		QDoubleSpinBox bridgeSegLenSB;
		QDoubleSpinBox regGridResSB;
		QDoubleSpinBox nodeConRadSB; 
		QDoubleSpinBox obstPenRadSB;
		
		// Obstacle Avoidance
		QGroupBox obstavoidGB;
		QRadioButton noavoidRadBtn; 
 		QRadioButton forceFieldRadBtn; 
 		QRadioButton configSpaceRadBtn;
 		QRadioButton controlRadBtn;  		

		// Command Actions
		QGroupBox actionGB;
		QPushButton captureBtn;
		QPushButton pathPlanBtn;
		QPushButton generateSpaceBtn;
		QPushButton pathFollowBtn;
		QPushButton loadMapBtn;	
		
		friend class NavContainer;
		static unsigned *image, *null;
        static int width, height, components;
		//static const int RobotNodeType = QTreeWidgetItem::UserType+1; 
		//static const int PatchNodeType = QTreeWidgetItem::UserType+2;
		//static const int SnapNodeType = QTreeWidgetItem::UserType+3;
	
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

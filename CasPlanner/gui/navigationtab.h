#ifndef NAVIGATIONTAB_H
#define NAVIGATIONTAB_H

#include <QWidget>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTreeWidget>
#include <QPushButton>
#include <QHash>
#include "mapviewer.h"
#include "MapPainter.h"
#include "robotmanager.h"
#include "Node.h"
#include "configfile.h"

class RobotManager;

class NavControlPanel: public QWidget 
{
Q_OBJECT
    public:  
		NavControlPanel(QWidget *parent = 0,RobotManager *rob=0);
	public slots:
		void updateMap(); 
		void handleSelection(); 
		void updateSelectedObject(double);
		void save();
		void captureMap(); 
		void exportHtml(); 
		void setToRoot(); 
    signals:
		void propsChanged();
    private: 
		RobotManager * robotManager;
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
};

class NavContainer : public QWidget
{
Q_OBJECT
    public:
		NavContainer(QWidget *parent = 0,RobotManager *robotManager=0);
		~NavContainer();
		MapPainter * mapPainter;
	public slots:
		void Plan();
		void LoadMap(); 
		void GenerateSpace();
		void Follow();
		//void RenderMap(Map & map);
    private:
       	Node * path;
    	RobotManager * robotManager;
		NavControlPanel navControlPanel; 
	friend class NavControlPanel; 
};

#endif

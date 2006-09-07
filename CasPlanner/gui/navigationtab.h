#ifndef NAVIGATIONTAB_H
#define NAVIGATIONTAB_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QWidget>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTreeWidget>
#include <QPushButton>
#include <QHash>
#include "mapviewer.h"
#include "mappainter.h"
#include "playground.h"
#include "Node.h"
#include "configfile.h"

class RobotManager;
class MapViewer;
class PlayGround;
class NavContainer;

class NavControlPanel: public QWidget 
{
Q_OBJECT
    public:  
		NavControlPanel(NavContainer *container,int currRobot);
    private: 
		NavContainer *navContainer;
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
 		QRadioButton vfhRadBtn;  		

		// Command Actions
		QGroupBox actionGB;
		QPushButton pauseBtn;
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
		NavContainer(QWidget *parent ,PlayGround *playGround);
		~NavContainer();
		MapPainter * mapPainter;
		MapViewer  * mapViewer;
	public slots:
		void updateMap(); 
		void updateSelectedObject(double);
		void updateSelectedAvoidanceAlgo(bool);
		void save();
		void captureMap(); 
		void exportHtml(); 
		void setToRoot(); 	
		void pathPlan();
		void loadMap(); 
		void generateSpace();
		void pathFollow();
		void Finished();
    private:
       	Node * path;
    	PlayGround * playGround;
		NavControlPanel navControlPanel; 
    	int currRobot;
		bool pause,following;
		friend class NavControlPanel; 
};

#endif

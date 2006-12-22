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
#include "playground.h"
#include "node.h"
#include "configfile.h"

class RobotManager;
class MapViewer;
class PlayGround;
class NavContainer;

using namespace CasPlanner;

class NavControlPanel: public QWidget
{
Q_OBJECT
    public:
		NavControlPanel(NavContainer *container,PlayGround *playG);
		void updateRobotSetting();
	public slots:
		void updateSelectedObject(double);
		void updateSelectedAvoidanceAlgo(bool);
		void updateSelectedRobot(bool);
		void save();
		void setNavigation();
		void exportHtml();
		void pathPlan();
		void loadMap();
		void generateSpace();
		void pathFollow();
		void pathTraversed();
		void setStart(Pose);
		void setEnd(Pose);
		void setMap(Map * map);
    private:
		NavContainer *navContainer;
		PlayGround * playGround;
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
		QVector <QRadioButton *> availableRobots;
		// Command Actions
		QGroupBox actionGB;
		QPushButton pauseBtn;
		QPushButton pathPlanBtn;
		QPushButton generateSpaceBtn;
		QPushButton pathFollowBtn;
		QPushButton captureImage;

		//Pointers to the currently selected Robot
		QGroupBox robotsGB;
		RobotManager *currRobot;
		QTreeWidgetItem *robotItem;
		bool robotInitialization;
		friend class NavContainer;
		static unsigned *image, *null;
       	Node * path;
        static int width, height, components;
		static const int AutonomousNav = QTreeWidgetItem::UserType+1;
		static const int ManualNav     = QTreeWidgetItem::UserType+2;
};

class NavContainer : public QWidget
{
Q_OBJECT
    public:
		~NavContainer();
		NavContainer(QWidget *parent ,PlayGround *playGround);
		MapViewer  * mapViewer;
	public slots:
//		void renderPath();
//		void setStart();
//		void setEnd();
    private:
    	PlayGround * playGround;
		RobotManager *currRobot;
		NavControlPanel navControlPanel;
		friend class NavControlPanel;
};

#endif

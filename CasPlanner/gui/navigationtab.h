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
		void startIntentionRecognition();
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
		QPushButton intentionRecognitionBtn;

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

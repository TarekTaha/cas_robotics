#ifndef SENSORSGUI_H
#define SENSORSGUI_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QtOpenGL>
#include <robotmanager.h>
#include <QImage>
#include <QPointer>
#include <QLabel>
#include <QGridLayout>
#include <QFileDialog>
#include <QWidget>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QTreeWidget>
#include <QPushButton>
#include <QGroupBox>
#include <QRadioButton>
#include <QObject>
#include <QHash>
#include <QTime>

#include "playground.h"
#include "voronoipathplanner.h"
#include "sensors.h"
#include "interfaceprovider.h"
#include "laserrender.h"
#include "speedrender.h"
#include "ogrender.h"
#include "mapviewer.h"
#include "task.h"
#include "mapskeleton.h"
#include "voronoidiagram.h"
#include "TestMRF.h"

class TasksGui;
class TasksControlPanel;
class QMessageBox;
class SpeedRender;

using namespace defs;
using namespace CasPlanner;

class TasksControlPanel: public QWidget
{
	Q_OBJECT
    public:
		TasksControlPanel(TasksGui *tasksGui,QWidget *tasksGui);
		void updateRobotSetting();
	public slots:
		void updateSelectedVoronoiMethod(bool);
//		void updateSelectedRobot(bool);
		void save();
		void exportHtml();
		void loadMap();
		void setMap(QImage);
		void taskSelected(int);
		void runRandomTasks();		
//		void taskClicked(QListWidgetItem * item);		
// 	signals:
//		void generateSkeleton();
    private:

		TasksGui *tasksGui;
		// BayesianNetwork Parameters
		QGroupBox randomTasksGB;
		QDoubleSpinBox numRandomRuns;

		// Voronoi Method
		QGroupBox voronoiGB;
		QRadioButton innerSkeletonBtn;
 		QRadioButton outerSkeletonBtn;

		QVector <QRadioButton *> availableRobots;

		// Command Actions
		QGroupBox   actionGB;
		QPushButton pauseBtn;
		QPushButton randomTasksBtn;
		QPushButton generateSkeletonBtn;
		QPushButton captureImage;

		//Pointers to the currently selected Robot
		QGroupBox tasksGB;
		QTreeWidgetItem *robotItem;
		QListWidget tasksList;

		QTreeWidget selectedRobot;
//		QHash<QTreeWidgetItem *, RobotManager *> widget2RobMan;
//		QHash<RobotManager *, QTreeWidgetItem *> robMan2Widget;
		friend class TasksGui;
		static unsigned *image, *null;
        static int width, height, components;
		static const int AutonomousNav = QTreeWidgetItem::UserType+1;
		static const int ManualNav     = QTreeWidgetItem::UserType+2;
};

class MapGL: public QGLWidget
{
	Q_OBJECT
    public:
// 		MapGL(QWidget *parent,SSkelPtr & sskel);
		MapGL(TasksGui *,QWidget *parent);
        void initializeGL();
        void renderSkeleton();
        void renderPath();
        void drawProbHisto(QPointF pos, double prob);
        void paintGL();
        void resizeGL(int w, int h);
        void setSSkelPtr(SSkelPtr sskel);
		QSize sizeHint();
        void config();
        QSize setMinimumSizeHint();
	public slots:
		void keyPressEvent(QKeyEvent *e);
    private:
		TasksGui *tasksGui;
		SSkelPtr  sskel;
		float zoomFactor;
		float xOffset, yOffset, zOffset;
		float yaw, pitch;
		float aspectRatio;
		float fudgeFactor;
		bool showGrids;
		bool firstTime;
		QTimer * renderTimer;
        int skeletonList;		
		friend class TasksGui;
};


class TasksGui :public QWidget
{
    Q_OBJECT
    public:
        TasksGui(QWidget *parent = 0,PlayGround *playG=0);
        ~TasksGui();
        virtual int config();
		void requestSnap();
  		void resetTab();
        void setRadMode(int mode);
		void loadTasks(string filename);        
    	VoronoiPathPlanner * voronoiPlanner;    
    	QVector <Task> tasks;
    	bool skeletonGenerated;
        int totalVisits;		    	    
    	PlayGround * playGround;
		Virtual_Voronoi_diagram_2*      vvd;    
		VoronoiDiagram *     cvd;	
    public slots:
        void updateData();
//         void mousePressEvent(QMouseEvent *me);
//         void mouseMoveEvent(QMouseEvent *me);
//         void mouseReleaseEvent(QMouseEvent *me);
//         void wheelEvent(QWheelEvent *we);
//         void keyPressEvent(QKeyEvent *ke);
//         void keyReleaseEvent(QKeyEvent *ke);
        void provideSpeed(double &speed, double &turnRate);
		void generateSkeleton();
        void renderLaser();
        void renderOG();
        void renderStatic();
    signals:
        void newData();
    private:
		QTabWidget *tabContainer;
		TasksControlPanel tasksControlPanel;
		SSkelPtr  sskel;
        MapGL mapGL;
//        MapViewer mapGL;
        double speed;
        double turnRatio;
        double startX, startY;
		double ptzPan;
		double ptzTilt;
		bool ptzEnabled;
        double radPerPixel;
        double msperWheel;
};

#endif

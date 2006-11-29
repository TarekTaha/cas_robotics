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

#include "mapskeleton.h"
#include "sensors.h"
#include "interfaceprovider.h"
#include "laserrender.h"
#include "speedrender.h"
#include "ogrender.h"
#include "mapviewer.h"

class TasksGui;
class TasksControlPanel;
class QMessageBox;
class SpeedRender;

using namespace defs;

class TasksControlPanel: public QWidget 
{
Q_OBJECT
    public:
		TasksControlPanel(TasksGui *tasksGui,QWidget *tasksGui);
		void updateRobotSetting();
	public slots:
		void updateSelectedObject(double);
		void updateSelectedVoronoiMethod(bool);
		void updateSelectedRobot(bool);
		void handleRobotSelection();		
		void save();
		void setNavigation(); 
		void exportHtml();
		void pathPlan();
		void loadMap(); 
		void pathFollow();
		void Finished();
		void setStart();
		void setEnd();
		void setMap(QImage);
//	signals:
//		void generateSkeleton();
    private:
		
		TasksGui *tasksGui;		
		// BayesianNetwork Parameters
		QGroupBox bayesianNetGB;
		QDoubleSpinBox distanceToVetix;
	
		// Voronoi Method
		QGroupBox voronoiGB;
		QRadioButton innerSkeletonBtn; 
 		QRadioButton outerSkeletonBtn; 
 		  		
		QVector <QRadioButton *> availableRobots;
		
		// Command Actions
		QGroupBox actionGB;
		QPushButton pauseBtn;
		QPushButton pathPlanBtn;
		QPushButton generateSkeletonBtn;
		QPushButton randomTasksBtn;
		QPushButton captureImage;	
		
		//Pointers to the currently selected Robot
		QGroupBox robotsGB;
		QTreeWidgetItem *robotItem; 
		
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
    public: 
        MapGL();
        void initializeGL();
        void renderSkeleton();
        void paintGL();
        void resizeGL(int w, int h);
        void setSSkelPtr(SSkelPtr sskel);
        void setRobotGUI(TasksGui *);
        QSize setSizeHint();
        void config(); 
        QSize setMinimumSizeHint(); 
       	double cursorCentreX;
      	double cursorCentreY; 
        double zoomCentreX;
	    double zoomCentreY; 
     	double srCentreX;
	    double srCentreY; 
	    double zoomSizeX;
        double zoomSizeY; 
	    double srSizeX; 
	    double srSizeY; 
    private:
        double desiredAspectRatio;
        TasksGui *tasksGui;
        SSkelPtr sskel; 
		friend class TasksGui;
};


class TasksGui :public QWidget 
{
    Q_OBJECT
    public:
        TasksGui(QWidget *parent = 0); 
        ~TasksGui(); 
        virtual int config();
		void requestSnap();
  		void resetTab();
        void setRadMode(int mode);
    public slots:
        void updateData();
        void mousePressEvent(QMouseEvent *me); 
        void mouseMoveEvent(QMouseEvent *me); 
        void mouseReleaseEvent(QMouseEvent *me);
        void wheelEvent(QWheelEvent *we); 
        void keyPressEvent(QKeyEvent *ke); 
        void keyReleaseEvent(QKeyEvent *ke);
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
        MapGL mapGL;
		MapSkeleton mapSkeleton;        
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

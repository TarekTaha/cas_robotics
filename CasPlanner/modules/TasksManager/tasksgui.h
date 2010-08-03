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
#ifndef SENSORSGUI_H
#define SENSORSGUI_H

#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>

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
#include "laserrender.h"
#include "speedrender.h"
#include "ogrender.h"
#include "mapviewer.h"
#include "task.h"

#include "MatrixUtils.h"
#include "BoundPairExec.h"
#include "zmdpMainConfig.h"

#include "Pomdp.h"

using namespace std;
using namespace zmdp;

class TasksGui;
class TasksControlPanel;
class QMessageBox;
class SpeedRender;

using namespace CasPlanner;

#define COLOR_LIGHT_BLUE_A(alpha)   glColor4f(0.0,0.7,0.7,alpha);
#define COLOR_SIENNAL_A(alpha)      glColor4f(1.0,0.51,0.278,alpha);
#define COLOR_GREEN_A(alpha)        glColor4f(0.0,0.7,0.0,alpha);
#define COLOR_YELLOW_A(alpha)       glColor4f(0.7,0.7,0.0,alpha);
#define COLOR_RED_A(alpha)          glColor4f(0.8,0.0,0.0,alpha);
#define COLOR_MAGENTA_A(alpha)      glColor4f(1.0,0.0,1.0,alpha);
#define COLOR_BLUE_A(alpha)         glColor4f(0.0,0.0,0.7,alpha);
#define COLOR_ORANGE_A(alpha)       glColor4f(1.0,0.65,0.0,alpha);
#define COLOR_DEEP_PINK_A(alpha)    glColor4f(1.0,0.078,0.576,alpha);
#define COLOR_DARK_RED_A(alpha)     glColor4f(1.0,0.0,0.0,alpha);

#define COLOR_LIGHT_BLUE   glColor3f(0.0,0.7,0.7);
#define COLOR_SIENNAL      glColor3f(1.0,0.51,0.278);
#define COLOR_GREEN        glColor3f(0.0,0.7,0.0);
#define COLOR_YELLOW       glColor3f(0.7,0.7,0.0);
#define COLOR_RED          glColor3f(0.8,0.0,0.0);
#define COLOR_MAGENTA      glColor3f(1.0,0.0,1.0);
#define COLOR_BLUE         glColor3f(0.0,0.0,0.7);
#define COLOR_ORANGE       glColor3f(1.0,0.65,0.0);
#define COLOR_DEEP_PINK    glColor3f(1.0,0.078,0.576);
#define COLOR_DARK_RED     glColor3f(1.0,0.0,0.0);

class TasksControlPanel: public QWidget
{
    Q_OBJECT
public:
    TasksControlPanel(TasksGui *,QWidget *);
    void updateRobotSetting();
        public slots:
    void updateSelectedVoronoiMethod(bool);
    void save();
    void exportHtml();
    void loadMap();
    void setMap(QImage);
    void taskSelected(int);
    void runRandomTasks();
private:

    TasksGui *tasksGui;

    // BayesianNetwork Parameters
    QGroupBox randomTasksGB;
    QSpinBox numRandomRuns;

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
    QPushButton testModelBtn;
    QPushButton clearAllBtn;
    QComboBox timeOfDay;

    //Pointers to the currently selected Robot
    QGroupBox tasksGB;
    QTreeWidgetItem *robotItem;
    QListWidget tasksList;

    QTreeWidget selectedRobot;
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
    MapGL(Map*,TasksGui *,QWidget *parent);
    void mousePressEvent(QMouseEvent *me);
    void mouseMoveEvent ( QMouseEvent * me );
    void wheelEvent( QWheelEvent * event );
    void initializeGL();
    void renderSkeleton();
    void renderPath();
    void drawProbHisto(QPointF pos, double prob);
    void paintGL();
    void resizeGL(int w, int h);
    void setMapSkeleton(MapSkeleton *);
    void loadTexture();
    void renderMap();
    void displayGrid();
    QSize sizeHint();
    void config();
    QSize setMinimumSizeHint();
    void setShowPaths(bool state){this->showPaths = state;}
    public slots:
        void keyPressEvent(QKeyEvent *e);
private:
    TasksGui *tasksGui;
    float zoomFactor;
    float xOffset, yOffset, zOffset;
    float yaw, pitch;
    float aspectRatio;
    float fudgeFactor;
    bool showGrids;
    bool firstTime;
    bool mainMapBuilt;
    bool showPaths;
    GLuint texId;
    QTimer * renderTimer;
    int skeletonList,mapList,morningList,earlyMorningList,afternoonList,nightList;
    MapSkeleton *mapSkeleton;
    Map *ogMap;
    float ratioW, ratioH;
    int newWidth,newHeight;
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
    MapGL & getMapGL(){return mapGL;}
    VoronoiPathPlanner * voronoiPlanner;
    QVector <Task> tasks;
    bool skeletonGenerated;
    int totalVisits;
    PlayGround * playGround;
    QString timeOfDay;
public slots:
    void updateData();
    void provideSpeed(double &speed, double &turnRate);
    void generateSkeleton();
    void testModel();
    void timeOfDayChanged(QString);
    void clearAll();
    void saveImage();
signals:
    void newData();
private:
    QTabWidget *tabContainer;
    TasksControlPanel tasksControlPanel;
    MapGL mapGL;
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

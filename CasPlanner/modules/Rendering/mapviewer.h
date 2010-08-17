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
#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QtOpenGL>
#include <GL/glut.h>
#include <QTime>
#include <cmath>
#include "mapmanager.h"
#include "playground.h"
#include "node.h"
#include "map.h"
#include "FreeType.h"
#include "settings.h"

class PlayGround;

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


class MapViewer : public QGLWidget
{
Q_OBJECT
    public:
        MapViewer(QWidget *parent=0,PlayGround *playG=0);
        ~MapViewer();
        void initializeGL();
        void paintGL();
        void resizeGL(int w, int h);
        QSize sizeHint();
        QSize minimumSizeHint();
        void keyPressEvent(QKeyEvent *e);
        void mousePressEvent(QMouseEvent *me);
        void mouseDoubleClickEvent(QMouseEvent *me);
        void mouseMoveEvent ( QMouseEvent * me );
        void wheelEvent( QWheelEvent * event );
        QPointF getOGLPos(double x, double y);
        QPointF getOGLPos(QPointF point);
        void mouseReleaseEvent(QMouseEvent *me);
        void focusInEvent(QFocusEvent *fe);
        void focusOutEvent(QFocusEvent *fe);
        void loadTexture();
        void renderRobot();
        void renderLaser();
        void renderPaths();
        void renderSearchSpace();
        void renderExpandedTree();
        void renderSpatialStates();
        void renderDestIndicators();
        void renderObservation();
        void renderAction();
        void drawProbHisto(QPointF pos, double prob);
        void setRobotsLocation();
        void displayGrid();
        void showIndicators();
        void drawCircle(float radius);
        void drawCircle(QPointF center,float radius);
        GLuint makeObject();
        QImage captureMap();
        virtual void setProvider(MapProvider *provider);
        void renderMap();
    public Q_SLOTS:
        void update();
        void setShowSearchSpaceSamples(bool state);
        void setShowSearchSpaceTree   (bool state);
        void setShowSearchTree        (bool state);
        void setShowPath              (bool state);
        void setShowRobotTrail        (bool state);
        void setShowOGs         (int state);
        void setShowSnaps       (int state);
        void setShowGrids       (int state);
        void setShowRobots      (int state);
        void setShowPointclouds (int state);
        void setShowPatchBorders(int state);
        void updateMap          (Map *newMap);
        void saveImage          ();
        void searchSpaceGenerated();
    Q_SIGNALS:
        void moveMOLeft();
        void moveMORight();
        void moveMOUp();
        void moveMODown();
        void yawMOPos();
        void yawMONeg();
        void setStart(Pose);
        void setEnd(Pose);
    private:
        int screenWidth,count,step;
        PlayGround *playGround;
        int screenHeight;
        float zoomFactor;
        float xOffset, yOffset, zOffset;
        float yaw, pitch;
        float aspectRatio;
        float fudgeFactor;
        bool showOGs;
        bool showSnaps;
        bool showLabels;
        bool showGrids;
        bool showRobots;
        bool showPointclouds;
        bool showPatchBorders;
        bool showSearchSpaceSamples;
        bool showSearchSpaceTree;
        bool showSearchTree;
        bool showPath;
        bool showRobotTrail;
        bool searchSpaceListCreated;
        bool hideGoals;
        freetype::font_data font2Render;
        bool start_initialized,end_initialized,mainMapBuilt;
        MapManager mapManager;
        Pose start,end,wayPoint,newLocation;
        QVector <Pose> robotsLocation;
        Map * ogMap;
        QColor clearColor;
        QPointF mousePos;
        QHash<QString, int> snapDLs;
        GLdouble modelMatrix[16];
        double position[3];
        int viewport[4],mapList,searchSpaceList;
        GLdouble projMatrix[16];
        QTimer * renderTimer;
        friend class MapControlPanel;
        GLuint texId;
        double RGB_COLOR[10][3];
        float ratioW, ratioH;
        int newWidth,newHeight;
};


#endif



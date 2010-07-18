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

#include <libplayerc++/playerc++.h>
#include <libplayerinterface/player.h>

#include <QtOpenGL>
#include <GL/glut.h>
#include <QTime>
//#include "interfaceprovider.h"
#include "mapmanager.h"
#include "playground.h"
#include "navigationtab.h"
#include "node.h"
//#include "robotrender.h"
#include "map.h"

#include "FreeType.h"

class PlayGround;
class NavControlPanel;

class MapViewer : public QGLWidget
{
Q_OBJECT
    public:
        MapViewer(QWidget *parent=0,PlayGround *playG=0,NavControlPanel *navControlPanel=0);
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
        void renderSearchSpaceTree();
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
    public slots:
        void update();
        void setShowOGs         (int state);
        void setShowSnaps       (int state);
        void setShowGrids       (int state);
        void setShowRobots      (int state);
        void setShowPointclouds (int state);
        void setShowPatchBorders(int state);
        void updateMap          (Map *newMap);
        void saveImage          ();
    signals:
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
        NavControlPanel *navControlPanel;
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
        int viewport[4],mapList;
        GLdouble projMatrix[16];
        QTimer * renderTimer;
        friend class MapControlPanel;
        GLuint texId;
        double RGB_COLOR[10][3];
        float ratioW, ratioH;
        int newWidth,newHeight;
};


#endif



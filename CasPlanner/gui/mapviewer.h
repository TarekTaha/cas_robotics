#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QtOpenGL>
#include <GL/glut.h>
#include "interfaceprovider.h"
#include "MapManager.h"
#include "robotmanager.h"
//#include "robotrender.h"
#include "map.h"

//class RobotRender;
class RobotManager;

class MapViewer : public QGLWidget
{
Q_OBJECT
    public:
        MapViewer(QWidget *parent=0,RobotManager *rob=0,QString map="");
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
		QPointF getOGLPos(double x, double y);
		QPointF getOGLPos(QPointF point);	
		void mouseReleaseEvent(QMouseEvent *me);
		void focusInEvent(QFocusEvent *fe);
		void focusOutEvent(QFocusEvent *fe);
		int  loadImage(QString name);
		QImage getImage();
		void  SetMapFileName(QString name);
		Pose getStart();
		Pose getEnd();
		void setMapName(QString name);
		void renderRobot();
		void renderLaser();	
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
		void setWayPoint        (Pose *wayPoint);
		
    signals:
		void moveMOLeft(); 
		void moveMORight(); 
		void moveMOUp(); 
		void moveMODown(); 
		void yawMOPos(); 
		void yawMONeg();
		
    private:
		int screenWidth,count,step;
		QVector <QPointF> trail;
	    RobotManager * robotManager;
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
		bool start_initialized,end_initialized;
		Pose start,end;
		QString mapName;
//		RobotRender * robotRender;
	    MapManager mapManager;
	    Pose wayPoint;
	    Map * mapData; 	
	    QColor clearColor;
	   	QImage image;
	   	QPointF mousePos;
		QHash<QString, int> snapDLs;
	    GLdouble modelMatrix[16];
	    double position[3];
	    int viewport[4];		
	    GLdouble projMatrix[16];	    
		friend class MapControlPanel; 
		GLuint texId; 
};


#endif



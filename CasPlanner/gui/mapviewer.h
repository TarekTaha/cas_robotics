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
		void getOGLPos(double x, double y);
		void mouseReleaseEvent(QMouseEvent *me);
		void focusInEvent(QFocusEvent *fe);
		void focusOutEvent(QFocusEvent *fe);
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
	    Map * mapData; 	
	    QColor clearColor;
	   	QImage image;
	   	QPointF mouseDouble;
		QHash<QString, int> snapDLs;
		friend class MapControlPanel; 
		GLuint texId; 
};


#endif



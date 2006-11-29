#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QtOpenGL>
#include <GL/glut.h>
#include "interfaceprovider.h"
#include "mapmanager.h"
#include "playground.h"
#include "navigationtab.h"
#include "Node.h"
//#include "robotrender.h"
#include "map.h"

#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoSeparator.h>

//class RobotRender;
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
		QPointF getOGLPos(double x, double y);
		QPointF getOGLPos(QPointF point);	
		void mouseReleaseEvent(QMouseEvent *me);
		void focusInEvent(QFocusEvent *fe);
		void focusOutEvent(QFocusEvent *fe);
		int  loadImage(QString name);
		QImage getImage();
		void  SetMapFileName(QString name);
		void setMapName(QString name);
		void renderRobot();
		void renderLaser();	
		void renderPaths();
		void renderSearchTree();
		void renderExpandedTree();
		void setRobotsLocation();
		GLuint makeObject();
		QImage captureMap();
        virtual void setProvider(MapProvider *provider);
        void renderMap();
   	   	QImage image;
    public slots:
		void update();
		void setShowOGs         (int state);
		void setShowSnaps       (int state);
		void setShowGrids       (int state);
		void setShowRobots      (int state);
		void setShowPointclouds (int state);
		void setShowPatchBorders(int state);
		void renderMapPatch     (Map * mapPatch);
		void updateMap          (Map *newMap);
		void saveImage          ();
    signals:
		void moveMOLeft(); 
		void moveMORight(); 
		void moveMOUp(); 
		void moveMODown(); 
		void yawMOPos(); 
		void yawMONeg();
		virtual void  setMap(QImage);		
		virtual void  setStart(Pose);
		virtual void    setEnd(Pose);		
		
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
		bool start_initialized,end_initialized,mainMapBuilt;
		QString mapName;
	    MapManager mapManager;
	    Pose start,end,wayPoint;
	    QVector <Pose> robotsLocation;
	    Map * mapData; 	
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
		double RGB[10][3];
};


#endif



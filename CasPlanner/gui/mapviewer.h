#ifndef MAPVIEWER_H
#define MAPVIEWER_H

#include <QtOpenGL>
#include <GL/glut.h>
#include "interfaceprovider.h"
#include "MapManager.h"
#include "map.h"

class MapViewer : public QGLWidget
{
Q_OBJECT
    public:
        MapViewer(QWidget *parent=0);
		~MapViewer();
        void initializeGL();
        void paintGL();
        void resizeGL(int w, int h);
        QSize sizeHint();
        QSize minimumSizeHint();
		void keyPressEvent(QKeyEvent *e);
		void mousePressEvent(QMouseEvent *me);
		void mouseReleaseEvent(QMouseEvent *me);
		void focusInEvent(QFocusEvent *fe);
		void focusOutEvent(QFocusEvent *fe);
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
    MapManager mapManager; 
    Map * mapData; 	
    QColor clearColor;
   	QImage image;
	QHash<QString, int> snapDLs;
	friend class MapControlPanel; 
	GLuint texId; 
};


#endif



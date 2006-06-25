
#ifndef OGRENDERER_H
#define OGRENDERER_H

#include <QtOpenGL>
#include <GL/glut.h>

#include "interfaceprovider.h"
class OGRenderer : public QGLWidget
{
Q_OBJECT
    public:
        OGRenderer(QWidget *parent=0);
		~OGRenderer();
        void initializeGL();
        void paintGL();
        void resizeGL(int w, int h);
        QSize sizeHint();
        QSize minimumSizeHint();
		void keyPressEvent(QKeyEvent *e);
		void focusInEvent(QFocusEvent *fe); 
		void focusOutEvent(QFocusEvent *fe);
        virtual void setProvider(MapProvider *provider);   
		QImage captureMap(); 
    public slots:
		void update(); 
		void setShowOGs(int state); 
		void setShowSnaps(int state); 
		void setShowGrids(int state); 
		void setShowRobots(int state); 
		void setShowPointclouds(int state); 
		void setShowPatchBorders(int state); 
    signals:
		void moveMOLeft(); 
		void moveMORight(); 
		void moveMOUp(); 
		void moveMODown(); 
		void yawMOPos(); 
		void yawMONeg();
    private:
	//void renderPointcloud(orca::PointcloudPtr, float in_fudgeFactor=1.0); 
	//void renderSnap(orca::SnapPtr, float in_fudgeFactor=1.0); 
	void renderOG(Map *mapData, bool highlighted=false); 
	MapProvider *provider; 
	//void renderRobots(QVector<RobotLocation *> robotLocation, bool renderRobot);
	int screenWidth;
    int screenHeight;  
	float zoomFactor; 
	float xOffset, yOffset, zOffset; 
	float yaw, pitch; 
	float aspectRatio; 
	float fudgeFactor; 
	int displayList; 
	bool showOGs;
	bool showSnaps;
	bool showLabels; 
	bool showGrids; 
	bool showRobots; 
	bool showPointclouds; 
	bool showPatchBorders; 
	QHash<QString, int> snapDLs;
	friend class MapControlPanel; 
	GLuint texId; 

};


#endif



#ifndef SENSORSGUI_H
#define SENSORSGUI_H


#include "sensors.h"
#include <QtOpenGL>
#include <robotmanager.h>
#include <QImage>
#include <QPointer>
#include "interfaceprovider.h"
#include "laserrender.h"
#include "speedrender.h"
#include "ogrender.h"
class SensorsGui;
class QMessageBox;
class SpeedRender;
class SensorsGLW: public QGLWidget 
{
    public: 
        SensorsGLW();
        void initializeGL();
        void paintGL();
        void resizeGL(int w, int h);
        void setRobotComms(RobotManager *);
        void setRobotGUI(SensorsGui *);
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
        LaserRender laser; 
        SpeedRender speedMeter;
        OGRenderer  ogRenderer;
        bool laserEnabled,mapEnabled,speedEnabled;
        RobotManager *robotManager;
        SensorsGui *sensorsGui; 
		friend class SensorsGui;
};


class SensorsGui: public Sensors, public SpeedProvider, public MapProvider
{
    Q_OBJECT
    public:
        SensorsGui(QWidget *parent = 0,RobotManager *commsMgr=0); 
        ~SensorsGui(); 
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
        void renderLaser();
        void renderOG();
        void renderStatic();
        Map  provideMap();
    signals: 
        void newData(); 
    private:
		QTabWidget *tabContainer;
        void configButtons(); 
		QRadioButton *OGRadBtn;
		QRadioButton *laserRadBtn;
		QRadioButton *staticRadBtn;
        //RobotManager *robotManager; 
        SensorsGLW sensorsGL, sGL2,sGL3;
        QWidget buttonWidget;
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

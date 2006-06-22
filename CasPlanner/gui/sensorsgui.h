/***************************************************************************
 *   Copyright (C) 2006 by Waleed Kadous   *
 *   waleed@width   
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef SENSORSGUI_H
#define SENSORSGUI_H


#include "sensors.h"
#include <QtOpenGL>
#include <robotcomm.h>
#include <QImage>
#include <QPointer>
#include "laserrender.h"
#include "interfaceprovider.h"
#include "speedrender.h"

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
        void setRobotComms(RobotComm *);
        void setRobotGUI(SensorsGui *);
        QSize setSizeHint();
        void config(ConfigFile *cf, int sectionid); 
        QSize setMinimumSizeHint(); 
        //void setMode(CamMode m); 
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
        // CamRender camera;
        double desiredAspectRatio;
        //OmniRender omniCam; 
		//CamRender wideCam; 
        //ThermalRender irCam; 
        LaserRender laser; 
        SpeedRender speedMeter; 
        //bool omniCamEnabled;
        //bool wideCamEnabled;
        //bool irCamEnabled;
        bool laserEnabled;
        bool speedEnabled;
        RobotComm *comms;
        SensorsGui *sensorsGui; 
		//bool showThermal; 
		//int irMinimum;
		//int irMaximum; 
		friend class SensorsGui;
};


class SensorsGui: public Sensors, public SpeedProvider
{
    Q_OBJECT
    public:
        SensorsGui(CommManager *commsMgr, QWidget *parent = 0); 
        //SensorsGui(QWidget *parent = 0);
        ~SensorsGui(); 
        virtual int config(ConfigFile *cf, int sectionid);
		void setRadMode(int mode);
		void requestSnap();
  		void resetTab();
    public slots:
        void updateData();
        void mousePressEvent(QMouseEvent *me); 
        void mouseMoveEvent(QMouseEvent *me); 
        void mouseReleaseEvent(QMouseEvent *me);
        void wheelEvent(QWheelEvent *we); 
        void keyPressEvent(QKeyEvent *ke); 
        void keyReleaseEvent(QKeyEvent *ke);
        void provideSpeed(double &speed, double &turnRate);
        void victimFound();
//		void switchToTele();
//		void switchToPaused();
//		void switchToAuto();

    signals: 
        void newData(); 
    private:
		QTabWidget *robotView;
        void configButtons(); 
		//QPushButton *confirmBtn;
		//QPushButton *rejectBtn;
		QRadioButton *teleRadBtn;
		QRadioButton *pausedRadBtn;
		QRadioButton *autoRadBtn;
        RobotComm *comms; 
        SensorsGLW glw;
        QWidget buttonWidget;
        double speed; 
        double turnRatio;
        double startX, startY; 
		double ptzPan;
		double ptzTilt;
		bool ptzEnabled;
        double radPerPixel;// = 0.001; 
        double msperWheel;// = 0.0005; 
};

#endif

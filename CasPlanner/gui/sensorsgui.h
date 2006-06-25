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
#include <robotmanager.h>
#include <QImage>
#include <QPointer>
#include "laserrender.h"
#include "interfaceprovider.h"
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
        bool laserEnabled,mapEnabled;
        bool speedEnabled;
        RobotManager *robotManager;
        SensorsGui *sensorsGui; 
		friend class SensorsGui;
};


class SensorsGui: public Sensors, public SpeedProvider, public MapProvider
{
    Q_OBJECT
    public:
        SensorsGui(RobotManager *commsMgr, QWidget *parent = 0); 
        ~SensorsGui(); 
        virtual int config();
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
        Map  provideMap();
    signals: 
        void newData(); 
    private:
		QTabWidget *tabContainer;
        void configButtons(); 
		//QPushButton *confirmBtn;
		//QPushButton *rejectBtn;
		QRadioButton *teleRadBtn;
		QRadioButton *pausedRadBtn;
		QRadioButton *autoRadBtn;
        RobotManager *robotManager; 
        SensorsGLW sensorsGL;
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

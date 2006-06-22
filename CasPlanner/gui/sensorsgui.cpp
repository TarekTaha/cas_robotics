/***************************************************************************
 *   Copyright (C) 2006 by Waleed Kadous   *
 *   waleed@width   *
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

#include "sensorsgui.h"
#include <QLabel>
#include <QGridLayout>


SensorsGLW::SensorsGLW():
    QGLWidget(QGLFormat(QGL::AlphaChannel)),
    comms(0),
//    omniCam(this), 
//    wideCam(this),  
//    irCam(this),  
    laser(this),
    speedMeter(this),
    desiredAspectRatio(1),
    cursorCentreX(0.53),
    cursorCentreY(0.51), 
    zoomCentreX(0.53), 
    zoomCentreY(0.51), 
    srCentreX(0.54),
    srCentreY(0.55), 
    zoomSizeX(0.08),
    zoomSizeY(0.08), 
    srSizeX(0.54),  
    srSizeY(0.38)
//    showThermal(true),
//    irMinimum(100),
//    irMaximum(255)
{
}

QSize SensorsGLW::setMinimumSizeHint()
{
    return QSize(400*desiredAspectRatio,400);  
}

QSize SensorsGLW::setSizeHint()
{
    return QSize(800*desiredAspectRatio,800);  
}

void SensorsGLW::initializeGL()
{
    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    glFlush();
}


void SensorsGLW::setRobotGUI(SensorsGui *gui)
{
    sensorsGui = gui;
}

void SensorsGLW::setRobotComms(RobotComm *Comms)
{
    comms = Comms;
//    //camera.setInverted(true);
//
//    // irCam.setProvider(comms); 
//    // irCam.setImgId(2); 
//    if(irCamEnabled){
//	irCam.setProvider(comms); 
//	irCam.setImgId( CAMERA_THERMAL0); 
//	irCam.setInverted(true); 
//	irCam.setMinHeat(irMinimum);
//	irCam.setMaxHeat(irMaximum);
//	connect(comms, SIGNAL(imgUpdate(unsigned int)), &irCam, SLOT(updateImg( unsigned int )));
    if(laserEnabled)
    {
        laser.setProvider(comms); 
        connect(comms, SIGNAL(newData()), &laser, SLOT(updateData()));
        laser.setId(0);
    } 
//    if(omniCamEnabled)
//    {
//        omniCam.setProvider(comms);
//        omniCam.setImgId(CAMERA_OMNI0); 
//        connect(comms, SIGNAL(imgUpdate(unsigned int)), &omniCam, SLOT(updateImg(unsigned int)));
//    }
//    if(wideCamEnabled)
//    {
//        qDebug("Wide enabled");
//        wideCam.setProvider(comms);
//        wideCam.setImgId(CAMERA_WIDE0);
//        connect(comms, SIGNAL(imgUpdate(unsigned int)), &wideCam, SLOT(updateImg(unsigned int)));
//
//    }
    speedMeter.setSpeedProvider(sensorsGui); 
    connect(sensorsGui, SIGNAL(newData()), &speedMeter, SLOT(updateData()));

}

void SensorsGLW::resizeGL(int w, int h)
{
    // glMatrixMode(GL_PROJECTION);    
    // glLoadIdentity();
    //width divided by height. 
    int newWidth = w;
    int newHeight = h;
    int xOffset = 0;
    int yOffset = 0;  
    //qDebug("Resize w = %d h = %d", w, h); 
    if((float) w/h > desiredAspectRatio)
    {
        newWidth = h*desiredAspectRatio; 
        xOffset = (w-newWidth)/2; 
    }
    else if((float) w/h < desiredAspectRatio)
    {
        newHeight=w/desiredAspectRatio; 
        yOffset = (h-newHeight)/2; 
    }
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity(); 
    glOrtho(0, desiredAspectRatio, 0, 1, -200, 200); 
    glMatrixMode(GL_MODELVIEW);
    glViewport( xOffset, yOffset, (GLint)newWidth, (GLint)newHeight );
}

// check for camera flags
void SensorsGLW::config(ConfigFile *cf, int sectionid)
{
//    omniCamEnabled = (bool) cf->ReadInt(sectionid, "omniCamEnabled", 1);
//    //OmniCam is always cam 0
//    wideCamEnabled = (bool) cf->ReadInt(sectionid, "wideCamEnabled", 1); 
//    irCamEnabled = (bool) cf->ReadInt(sectionid, "irCamEnabled", 1); 
    laserEnabled = (bool) cf->ReadInt(sectionid, "laserEnabled", 1); 
    speedEnabled = (bool) cf->ReadInt(sectionid, "speedEnabled", 1); 
    if(speedEnabled)
    {
		speedMeter.setMaxSpeed(2); 
		speedMeter.setMaxTurnRate(2);
    }
}

void SensorsGLW::paintGL()
{
    //qDebug("RMV asked to paint image"); 
    //int displayList = glGenLists(1); 
    //glNewList(displayList, GL_COMPILE); 
    //qDebug("Mode is: %d", mode); 
    double camPanelRatio = 1.0; 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
//    if(wideCamEnabled)
//    {
//       glPushMatrix();
//       glTranslatef(0,0.2,0); 
//       glScalef(camPanelRatio,0.8,1);
//       glTranslatef(0.5,0.5,0); 
//       glScalef(-1,1,1);
//       glRotatef(180,0,0,1); 
//       glTranslatef(-0.5,-0.5,0); 
//       //qDebug("Rendering widecam"); 
//       wideCam.render();   
//       glPopMatrix(); 
//     }
//    if(omniCamEnabled){
//       //qDebug("Rendering ptzCam"); 
//       glPushMatrix();
//       glTranslatef(0,0,0); 
//       glScalef(camPanelRatio,0.2,1);
//       omniCam.render(); 
//       glPopMatrix();
//     }
     /*if(mode == CamNormal){
         glColor4f(0.0f,1.0f,0.0f,1.0f); 
         glBegin(GL_LINE_LOOP); 
         glVertex2f(0.4*camPanelRatio,0.4*0.8 + 0.2); 
         glVertex2f(0.4*camPanelRatio,0.6*0.8 + 0.2);
         glVertex2f(0.6*camPanelRatio,0.6*0.8 + 0.2);
         glVertex2f(0.6*camPanelRatio,0.4*0.8 + 0.2); 
         glEnd();
     }
     else{
         glPushMatrix(); 
         glTranslatef(0.3*camPanelRatio,0.3*0.8,0); 
         glScalef(0.5,0.5,1);
         zoomCam.render();
         glPopMatrix(); 
     }*/
//     if(irCamEnabled && showThermal)
//     {
//	 glPushMatrix();
//	 glTranslatef(0.25,0.38,0); 
//	 glScalef(0.65,0.65,1);
//	 glTranslatef(0.5,0.5,0.5);
//	 glRotatef(180,0,0,1); 
//	 glTranslatef(-0.5,-0.5,-0.5);  
//	 irCam.render();
//	 glPopMatrix(); 
//     }
     if(laserEnabled)
     {
         glPushMatrix(); 
         glTranslatef(camPanelRatio*.7,0.25, 0);
         glScalef(0.2,0.2,1);
         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
         glEnable(GL_POLYGON_SMOOTH); 
         glEnable(GL_BLEND);
         laser.render();
         glDisable(GL_BLEND);
         glDisable(GL_POLYGON_SMOOTH);
         glPopMatrix(); 
     }

     if(speedEnabled)
     {
         glPushMatrix();
         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
         glEnable(GL_POLYGON_SMOOTH); 
         glEnable(GL_BLEND);
         glTranslatef(camPanelRatio*0.75, 0.75,0);
         glScalef(camPanelRatio*0.2,0.2,1); 
         speedMeter.render(); 
         glDisable(GL_BLEND);
         glDisable(GL_POLYGON_SMOOTH);
         glPopMatrix(); 
     }
     glFlush();
}



void SensorsGui::updateData()
{
    
}
//SensorsGui::SensorsGui(QWidget *parent): 
//    Sensors(commsMgr, parent),
//	   robotView((QTabWidget*) parent),
//       speed(0.15), 
//	   turnRatio(5),
//	   ptzPan(0),
//	   ptzTilt(0),
//	   radPerPixel(0.001),
//	   msperWheel(0.0005)
//{
//    QVBoxLayout *layout = new QVBoxLayout();
//    layout->addWidget(&glw,4); 
//    layout->addWidget(&buttonWidget, 1);
//    setLayout(layout); 
//    updateGeometry();
//    setFocusPolicy(Qt::StrongFocus);
//}
SensorsGui::SensorsGui(CommManager *commsMgr, QWidget *parent): 
    Sensors(commsMgr, parent),
	   robotView((QTabWidget*) parent),
       speed(0.15), 
	   turnRatio(5),
	   ptzPan(0),
	   ptzTilt(0),
	   radPerPixel(0.001),
	   msperWheel(0.0005)
{
    QVBoxLayout *layout = new QVBoxLayout();
    layout->addWidget(&glw,4); 
    layout->addWidget(&buttonWidget, 1);
    setLayout(layout); 
    updateGeometry();
    setFocusPolicy(Qt::StrongFocus);
}

void SensorsGui::configButtons()
{
    QHBoxLayout *hLayout = new QHBoxLayout;
    teleRadBtn = new QRadioButton("&Teleop");
    pausedRadBtn = new QRadioButton("&Paused");
    autoRadBtn = new QRadioButton("Autonomous");

    //confirmBtn = new QPushButton("&Confirm Victim");
    //rejectBtn = new QPushButton("&Reject Victim");
    //hLayout->addWidget( confirmBtn, 1);
    //hLayout->addWidget( rejectBtn, 1);

    hLayout->addWidget( teleRadBtn, 1 );
    hLayout->addWidget( pausedRadBtn, 1 );
    hLayout->addWidget( autoRadBtn, 1 );

    // start at teleoperation mode
    teleRadBtn->setChecked(true);
    buttonWidget.setLayout(hLayout);
}

void SensorsGui::mousePressEvent(QMouseEvent *me)
{
    qDebug("Mouse pressed"); 
    startX = me->x();
    startY = me->y();
}

void SensorsGui::mouseReleaseEvent(QMouseEvent *me)
{
}

void SensorsGui::provideSpeed( double &in_speed, double &in_turnRate)
{
    in_speed = speed; 
    in_turnRate = turnRatio * speed;
}

void SensorsGui::mouseMoveEvent(QMouseEvent *me)
{
    startX = me->x(); 
    startY = me->y(); 
//  qDebug("deltaX = %d deltaY = %d", deltaX, deltaY);
//    if(glw.getMode() == CamZoom){
//        PtzValue desiredPtz(startPtz.pan+deltaX*0.003, startPtz.tilt - deltaY*0.003, startPtz.zoom);
//        comms->setPtz( desiredPtz);
//    }
//    else {
//        glw.moveDrivePan(-0.002*deltaX);  
//        glw.updateGL();
//    }
    if(ptzEnabled)
    {
		int deltaX = me->x() - startX; 
		int deltaY = me->y() - startY; 
		ptzPan -= deltaX*radPerPixel; 
		ptzTilt += deltaY*radPerPixel; 
		comms->setPtz(ptzPan, ptzTilt);  
	    //qDebug("Intermediate arm pos: %f %f %f", currentArmPos[0], currentArmPos[1], currentArmPos[2]);
		startX = me->x(); 
		startY = me->y(); 
    }
}


void SensorsGui::keyPressEvent(QKeyEvent *e)
{
    int index;
    QString text;
    bool ok;

    //qDebug("Key pressed %s", qPrintable(e->text()));
        if(e->key() == Qt::Key_Up)
        {
            qDebug("Cursor moved up"); 
            //emit cursorMove(0,1); 
        }
        else if(e->key() == Qt::Key_Down)
        {
            //emit cursorMove(0,-1); 
        }
        else if(e->key() == Qt::Key_Left)
        {
            //emit cursorMove(-1,0); 
        }
        else if(e->key() == Qt::Key_Right)
        {
            //emit cursorMove(1,0); 
        }
        switch(e->text().at(0).toAscii())
        {
            case 'w': 
                comms->setSpeed(speed); 
                break;
            case 'a':
                comms->setTurnRate(turnRatio*speed); 
                break;
            case 's':
                comms->setSpeed(-speed);
                break;
            case 'd':
                comms->setTurnRate(-turnRatio*speed); 
                break;
            case 'z':
 
            case '.':
                //emit cursorCentre();
                break;
	    case 'n':
		// Start new patch 
		//comms->requestNewPatch(0); 
		break;
            case 'v':
	       text = QInputDialog::getText(this, "RescueGUI: Victim label", "Enter a label for the victim:",
               QLineEdit::Normal, QString::null, &ok);
		if ( ok ) {
		    if(text.isEmpty()){
    			text = "Unlabelled"; 
		    }
//		    orca::CartesianPoint2d centre;
//		    orca::Size2d size; 
//		    centre.x = (glw.cursorCentreX-glw.srCentreX)/glw.srSizeX+0.5;
//		    centre.y = (glw.cursorCentreY-glw.srCentreY)/glw.srSizeY+0.5;
//		    size.w = glw.zoomSizeX/glw.srSizeX; 
//		    size.l = glw.zoomSizeY/glw.srSizeY; 
//		    
//		    comms->requestSnap(orca::SNAPVICTIM, text, centre, size);

        int index = robotView->indexOf(this);
        // set Homer tab title
        robotView->setTabText(index, "HOMER");
    
        // set Victim Signal icon
        robotView->setTabIcon(index, QIcon("../ui/rescuegui-branchCommsRefactor/blank.xpm"));
        //comms->setCtrlMode(true);
		}
                break;
	    case 't':
//		glw.showThermal = !glw.showThermal;
//		break; 
//	    case '[':
//		glw.irMinimum--;
//		glw.irMaximum--;
//		glw.irCam.setMinHeat(glw.irMinimum);
//		glw.irCam.setMaxHeat(glw.irMaximum);  
//		break;
//	    case ']':
//		glw.irMinimum++;
//		glw.irMaximum++; 
//		glw.irCam.setMinHeat(glw.irMinimum);
//		glw.irCam.setMaxHeat(glw.irMaximum);  
                //emit irThresh(1); 
		break;
            case 'l':
                //
                break;
            case 'c':
                //emit newMode(linkmode); 
                break;
            case 'm': 
                //mouseInvert = (-1)*mouseInvert; 
                break;
            case 'I':
                //emit cancelMode();
                break;
            case 'f':
                //if(isFullScreen()){
                //    fullScreenOff(); 
                //}
                //else {
                //    fullScreenOn();
               // }
                break;
            case 'h':
                //hideHUD = !hideHUD; 
                break; 
            case 'x':
                //hideExtras = !hideExtras; 
                break; 
            case  '\'':
                break;
            case '/':
                break; 
            case '1':
            case '2': 
            case '3':
            case '4':
            case '5':
            case '6':
            case '7':
            case '8': 
            case '9':
            case '0':
                break; 
            case 'b':
                break; 
            default:
                e->ignore(); 
        }

}

void SensorsGui::keyReleaseEvent(QKeyEvent *e)
{
    qDebug("Key released: %s autorepeat: %d", qPrintable(e->text()), e->isAutoRepeat()); 
    if(!e->isAutoRepeat()){
        /* emit allStop(); */
        switch(e->text().at(0).toAscii()){
        case 'w': 
        case 's':
            comms->setSpeed(0); 
            break;
        case 'a':
        case 'd':
            comms->setTurnRate(0); 
            break;
        }
    }
}
 
void SensorsGui::wheelEvent( QWheelEvent *we)
{
    // check for mouse move direction
    int deltaMoved = we->delta();
    if( deltaMoved < 0 )
        speed -= 0.01; // decrease Homer speed a bit
    else
        speed += 0.01; // increase speed
    // emit newData(); 
    //startPtz = comms->getPtz(); 
    //PtzValue desiredPtz(startPtz.pan, startPtz.tilt, startPtz.zoom+0.001*we->delta()); 
    //comms->setPtz( desiredPtz); 
}

SensorsGui::~SensorsGui()
{
}

int SensorsGui::config(ConfigFile *cf, int sectionid)
{
    QString commsName = cf->ReadString(sectionid, "homer", "homer"); 
    glw.setRobotGUI(this); 
    glw.config(cf, sectionid);
    // Now retrieve the appropriate comms. 
  //  comms = qobject_cast<comms *>(commsMgr->getCommsByName(commsName)); 
    glw.setRobotComms(comms);

    // config buttons
    configButtons();

    // connnect victim found signal
    connect(comms, SIGNAL(victimFound()), this, SLOT(victimFound()));

    // connect confirm and reject btns
    //connnect(confirmBtn, SIGNAL(clicked()), this, SLOT(confirmVictim()));
    //connect(rejectBtn, SIGNAL(clicked()), this, SLOT(rejectVictim()));

    // signals for changing modes
    connect( teleRadBtn, SIGNAL(clicked()), this, SLOT(switchToTele()));
    connect( pausedRadBtn, SIGNAL(clicked()), this, SLOT(switchToPaused()));
    connect( autoRadBtn, SIGNAL(clicked()), this, SLOT(switchToAuto()));

    ptzEnabled = cf->ReadInt(sectionid, "ptzEnabled", 1);
    comms->start();
    return 1; 

}

/*
 * changing mode slots
 * modes are defined in autocontroller.h
 * enum AutoControllerMode { AUTO_TELEOP, AUTO_WAYPOINT, AUTO_FULL, AUTO_VICTIMFOUND, AUTO_PAUSED };  
 */
//void SensorsGui::switchToTele(){
//    resetTab();
//    comms->setControlMode(0); // AUTO_TELEOP
//}
//
//void SensorsGui::switchToPaused(){
//    resetTab();
//    comms->setControlMode(4); // AUTO_PAUSED
//}
//
//void SensorsGui::switchToAuto(){
//    resetTab();
//    comms->setControlMode(2); // AUTO_FULL
//}

/*
 * set Checked status for radio buttons when modes received
 */
void SensorsGui::setRadMode(int mode)
{
    switch(mode){
        case 0: // AUTO_TELEOP
            teleRadBtn->setChecked(true);
            break;
        case 4: // AUTO_PAUSED
            pausedRadBtn->setChecked(true);
            break;
        case 2: // AUTO_FULL
            autoRadBtn->setChecked(true);
            break;
        default:
	    qDebug("Mode is incorrect");
    }
}

/*
// we either have to do the separation here or in comms,
// but doing it here will avoid further complication
void SensorsGui::confirmVictim(){
    QString text;
    bool ok;
    qDebug("HOMER victim found confirmed clicked");

    //comms->setCtrlMode(true);
    
    text = QInputDialog::getText(this, "RescueGUI: Victim label", "Enter a label for the victim:",
    QLineEdit::Normal, QString::null, &ok);
    if ( ok ) {
      if(text.isEmpty()){
	        text = "Unlabelled"; 
      }
      orca::CartesianPoint2d centre;
      orca::Size2d size; 
      centre.x = (glw.cursorCentreX-glw.srCentreX)/glw.srSizeX+0.5;
      centre.y = (glw.cursorCentreY-glw.srCentreY)/glw.srSizeY+0.5;
      size.w = glw.zoomSizeX/glw.srSizeX; 
      size.l = glw.zoomSizeY/glw.srSizeY; 
		    
      comms->requestSnap(orca::SNAPVICTIM, text, centre, size);
    }
    
    int index = robotView->indexOf(this);
    // set Homer tab title
    robotView->setTabText(index, "HOMER");
    
    // set Victim Signal icon
    robotView->setTabIcon(index, QIcon("../ui/rescuegui-branchCommsRefactor/blank.xpm"));
    
    

}
*/
/*
void SensorsGui::rejectVictim()
{

    qDebug("HOMER victim found reject clicked");

    int index = robotView->indexOf(this);
    // set Homer tab title
    robotView->setTabText(index, "HOMER");

    // set Victim Signal icon
    robotView->setTabIcon(index, QIcon("../ui/rescuegui-branchCommsRefactor/blank.xpm"));
    comms->setCtrlMode(false);
}
*/
void SensorsGui::victimFound()
{
    // get this tab index
    int index = robotView->indexOf(this);

    // set Homer tab title
    robotView->setTabText(index, "HOMER - Victim found");

    // set Victim Signal icon
    robotView->setTabIcon(index, QIcon("../ui/rescuegui-branchCommsRefactor/vicsignal.xpm"));

    // change check box to AUTO_PAUSED
    setRadMode(4);
    // request snap
    // requestSnap();

    qDebug("HOMER victim found signal received");
}

void SensorsGui::resetTab()
{
    // get this tab index
    int index = robotView->indexOf(this);

    // set Homer tab title
    robotView->setTabText(index, "HOMER");

    // set Victim Signal icon
    robotView->setTabIcon(index, QIcon("../ui/rescuegui-branchCommsRefactor/blank.xpm"));
}

void SensorsGui::requestSnap()
{
//    orca::CartesianPoint2d centre;
//    orca::Size2d size; 
//    centre.x = (glw.cursorCentreX-glw.srCentreX)/glw.srSizeX+0.5;
//    centre.y = (glw.cursorCentreY-glw.srCentreY)/glw.srSizeY+0.5;
//    size.w = glw.zoomSizeX/glw.srSizeX; 
//    size.l = glw.zoomSizeY/glw.srSizeY; 
//		    
//    comms->requestSnap(orca::SNAPVICTIM, "Homer Victim Found", centre, size);
}

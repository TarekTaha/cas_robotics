#include "sensorsgui.h"
#include <QLabel>
#include <QGridLayout>


SensorsGLW::SensorsGLW():
    QGLWidget(QGLFormat(QGL::AlphaChannel)),
    robotManager(0),
    laser(this),
    speedMeter(this),
//    ogRenderer(this),
    desiredAspectRatio(1.33),
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
{
}

QSize SensorsGLW::setMinimumSizeHint()
{
    return QSize(80*desiredAspectRatio,60);  
}

QSize SensorsGLW::setSizeHint()
{
    return QSize(160*desiredAspectRatio,120);  
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

void SensorsGLW::setRobotComms(RobotManager *rob)
{
    robotManager = rob;
    if(robotManager->commManager->connected)
    {
	    if(laserEnabled)
	    {
	        laser.setProvider(robotManager->commManager); 
	        connect(robotManager->commManager, SIGNAL(newData()), &laser, SLOT(updateData()));
	        laser.setId(0);
	    }
	    if(mapEnabled)
	    {
//	        mapViewer.setProvider(robotManager->commManager);
//	        connect(robotManager->commManager, SIGNAL(newData()), &mapViewer, SLOT(update()));
	    }
	    speedMeter.setSpeedProvider(sensorsGui); 
	    connect(robotManager->commManager, SIGNAL(newData()), &speedMeter, SLOT(updateData()));
    }

}

void SensorsGLW::resizeGL(int w, int h)
{
    // glMatrixMode(GL_PROJECTION);    
    // glLoadIdentity();
    // width divided by height. 
    int newWidth = w;
    int newHeight = h;
    int xOffset = 0;
    int yOffset = 0;  
    qDebug("Resize w = %d h = %d", w, h); 
    if((float) w/h > desiredAspectRatio)
    {
        newWidth = (int)h*desiredAspectRatio; 
        xOffset = (w-newWidth)/2; 
    }
    else if((float) w/h < desiredAspectRatio)
    {
        newHeight= (int)w/desiredAspectRatio; 
        yOffset = (h-newHeight)/2; 
    }
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity(); 
    glOrtho(0, desiredAspectRatio, 0, 1, -200, 200); 
    //glOrtho(-1, 1, -1, 1, 0, 0); 
    glMatrixMode(GL_MODELVIEW);
    glViewport( xOffset, yOffset, (GLint)newWidth, (GLint)newHeight );
}

void SensorsGLW::config()
{
    //laserEnabled = true;//(bool) cf->ReadInt(sectionid, "laserEnabled", 1); 
    //speedEnabled = true;//(bool) cf->ReadInt(sectionid, "speedEnabled", 1);
    //mapEnabled   = true;//(bool) cf->ReadInt(sectionid, "mapEnabled", 1);
    if(speedEnabled)
    {
		speedMeter.setMaxSpeed(2); 
		speedMeter.setMaxTurnRate(2);
    }
}

void SensorsGLW::paintGL()
{
    double camPanelRatio = 1.0; 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
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
     if(mapEnabled)
     {
	   //  mapViewer.render();
     }     
     glFlush();
}

SensorsGui::SensorsGui(QWidget *parent,RobotManager *rob): 
       Sensors(parent,rob),
	   tabContainer((QTabWidget*) parent),
       speed(0.15), 
	   turnRatio(5),
	   ptzPan(0),
	   ptzTilt(0),
	   radPerPixel(0.001),
	   msperWheel(0.0005)
{
    mapViewer = new MapViewer(parent);
    mapViewer->setProvider(rob->commManager);
    //connect(rob->commManager, SIGNAL(newData()), mapViewer, SLOT(update()));    
    
    QHBoxLayout *layout3 = new QHBoxLayout; 
    QVBoxLayout *layout1 = new QVBoxLayout(),*layout2 = new QVBoxLayout(),*layout4 = new QVBoxLayout();
    layout1->addWidget(mapViewer,1);
   // layout2->addWidget(&sGL2,1);
    //layout2->addWidget(&sensorsGL,1);
    layout3->addLayout(layout1);
    //layout3->addLayout(layout2);
    QVBoxLayout *layout = new QVBoxLayout();
    //layout4->addWidget(&buttonWidget, 1);
    layout->addLayout(layout3);
    layout->addLayout(layout4);
    setLayout(layout); 
    updateGeometry();
    setFocusPolicy(Qt::StrongFocus);
    config();
}

void SensorsGui::updateData()
{
    
}

int SensorsGui::config()
{
    QString commsName ="Wheelchair";
    sensorsGL.setRobotGUI(this); 
    sensorsGL.config();
    sensorsGL.setRobotComms(robotManager);
    //configButtons();
    // signals for changing modes
    //connect( OGRadBtn, SIGNAL(clicked()), this, SLOT(renderOG()));
    //connect( laserRadBtn, SIGNAL(clicked()), this, SLOT(renderLaser()));
    //connect( staticRadBtn, SIGNAL(clicked()), this, SLOT(renderStatic()));
    // robotManager->commManager->start();
    return 1; 

}

void SensorsGui::configButtons()
{
    QHBoxLayout *hLayout = new QHBoxLayout;
    OGRadBtn =   new QRadioButton("&OG-MAP");
    laserRadBtn = new QRadioButton("&Laser");
    staticRadBtn =   new QRadioButton("StaticMap");

    hLayout->addWidget( OGRadBtn, 1 );
    hLayout->addWidget( laserRadBtn, 1 );
    hLayout->addWidget( staticRadBtn, 1 );

    OGRadBtn->setChecked(true);
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
    //sensorsGL.moveDrivePan(-0.002*deltaX);  
    sensorsGL.updateGL();
    if(ptzEnabled)
    {
		int deltaX = int(me->x() - startX); 
		int deltaY = int(me->y() - startY); 
		ptzPan -= deltaX*radPerPixel; 
		ptzTilt += deltaY*radPerPixel; 
		robotManager->commManager->setPtz(ptzPan, ptzTilt);  
	    //qDebug("Intermediate arm pos: %f %f %f", currentArmPos[0], currentArmPos[1], currentArmPos[2]);
		startX = me->x(); 
		startY = me->y(); 
    }
}


void SensorsGui::keyPressEvent(QKeyEvent *e)
{
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
                robotManager->commManager->setSpeed(speed); 
                break;
            case 'a':
                robotManager->commManager->setTurnRate(turnRatio*speed); 
                break;
            case 's':
                robotManager->commManager->setSpeed(-speed);
                break;
            case 'd':
                robotManager->commManager->setTurnRate(-turnRatio*speed); 
                break;
            case 'z':
 
            case '.':
                //emit cursorCentre();
                break;
	    case 'n':
		// Start new patch 
		//robotManager->commManager->requestNewPatch(0); 
		break;
            case 'v':
	       		text = QInputDialog::getText(this, "Name Dialog", "Please Enter your name: ",
               	QLineEdit::Normal, QString::null, &ok);
				if ( ok ) 
				{
		    		if(text.isEmpty())
		    		{
    					text = "NoName"; 
		    		}
			        int index = tabContainer->indexOf(this);
			        tabContainer->setTabText(index, "CAS PLANNER");
				}
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
    if(!e->isAutoRepeat())
    {
        // Stop 
        switch(e->text().at(0).toAscii())
        {
        case 'w': 
        case 's':
            robotManager->commManager->setSpeed(0); 
            break;
        case 'a':
        case 'd':
            robotManager->commManager->setTurnRate(0); 
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
}

SensorsGui::~SensorsGui()
{
}

void SensorsGui::renderOG()
{
    //resetTab();
	setRadMode(0);
}

void SensorsGui::renderLaser()
{
    //resetTab();
	setRadMode(1);
}
void SensorsGui::renderStatic()
{
    //resetTab();
	setRadMode(2);
}

void SensorsGui::setRadMode(int mode)
{
	qDebug("Radio Button Clicked");
    switch(mode)
    {
        case 0: // AUTO_TELEOP
        	sensorsGL.mapEnabled   = true;
        	sensorsGL.laserEnabled = false;
        	sensorsGL.speedEnabled = false;
            OGRadBtn->setChecked(true);
            qDebug("OG MAP enabled");
            break;
        case 1: // AUTO_PAUSED
            sensorsGL.mapEnabled   = false;
        	sensorsGL.laserEnabled = true;
        	sensorsGL.speedEnabled = false;
            laserRadBtn->setChecked(true);
            qDebug("Laser Enabled");
            break;
        case 2: // AUTO_FULL
           	sensorsGL.mapEnabled   = false;
        	sensorsGL.laserEnabled = false;
        	sensorsGL.speedEnabled = true;
            staticRadBtn->setChecked(true);
            qDebug("Speed Enabled");            
            break;
        default:
	    qDebug("Mode is incorrect");
    }
}

void SensorsGui::resetTab()
{
    // get this tab index
    int index = tabContainer->indexOf(this);

    // set tab title
    tabContainer->setTabText(index, "Cas Planner");

    // set Victim Signal icon
    //tabContainer->setTabIcon(index, QIcon("../ui/rescuegui-branchCommsRefactor/blank.xpm"));
}

Map SensorsGui::provideMap()
{
	return robotManager->commManager->provideMap();
}
void SensorsGui::requestSnap()
{

}

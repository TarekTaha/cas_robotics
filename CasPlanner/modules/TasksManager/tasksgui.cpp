#include "tasksgui.h"

TasksControlPanel::TasksControlPanel(TasksGui *tasksGui,QWidget *parent):
	QWidget(parent),
	tasksGui(tasksGui),
	bayesianNetGB("Bayesian Network Parameters"),
	distanceToVetix(),
	voronoiGB("Map Voronoi Skeleton"),
	innerSkeletonBtn("Inner Skeleton"),
	outerSkeletonBtn("Outer Skeleton"),
	actionGB("Action"),
	pauseBtn("Pause"), 
	pathPlanBtn("Random Tasks"),
	generateSkeletonBtn("Generate Skeleton"), 
	randomTasksBtn("Random Tasks"),
	captureImage("Capture Image"),
	robotsGB("Set of Tasks")
{
//	qDebug("Initializing Control Panel"); fflush(stdout);	
//	robMan2Widget.clear();
//	widget2RobMan.clear();
//	QRadioButton *rob;
//    for(int i=0; i < navContainer->playGround->robotPlatforms.size(); i++)
//	{
//		temp = playGround->robotPlatforms[i]; 
//		if(!robMan2Widget.contains(temp))
//		{
//			qDebug("Adding %s",qPrintable(temp->robot->robotName));	
//		    robotItem = new QTreeWidgetItem(QStringList(QString("Robot: ").append(temp->robot->robotName)),AutonomousNav);
//		    robMan2Widget[temp] = robotItem;
//		    widget2RobMan[robotItem] = temp; 
//		    selectedRobot.addTopLevelItem(robotItem);
//		    selectedRobot.expandItem(robotItem);
//		}
//		else
//		{
//		    robotItem = robMan2Widget[temp];  
//		}
//		rob = new QRadioButton(QString("Robot: ").append(temp->robot->robotName));
//		availableRobots.push_back(rob);
//	}
//	currRobot = playGround->robotPlatforms[0]; 
//	if(!currRobot)
//	{
//		qDebug("WHAT THEEEE !!!");
//		exit(1);
//	}
	
//    selectedRobot.setColumnCount(1); 
//    selectedRobot.setHeaderLabels(QStringList("Available Robots")); 
    QVBoxLayout *hlayout = new QVBoxLayout;
//    hlayout->addWidget(&selectedRobot,1);
    hlayout->addWidget(&robotsGB,1);
    hlayout->addWidget(&bayesianNetGB,1);
    hlayout->addWidget(&voronoiGB,1);    
    hlayout->addWidget(&actionGB,1); 
    this->setLayout(hlayout);
    
  
    QGridLayout *parLayout = new QGridLayout;
    parLayout->addWidget(new QLabel("Distance 2 Vertix"),0,0);
    parLayout->addWidget(&distanceToVetix,0,1); 
    bayesianNetGB.setLayout(parLayout);
	
	//Loading Default values from config file
    distanceToVetix.setMinimum(0); 
    distanceToVetix.setMaximum(1);
	distanceToVetix.setSingleStep(0.01);
	distanceToVetix.setValue(0.5);

    QVBoxLayout *showL = new QVBoxLayout; 
    showL->addWidget(&innerSkeletonBtn);
    showL->addWidget(&outerSkeletonBtn);
    innerSkeletonBtn.setChecked(true);
	updateSelectedVoronoiMethod(true);
    voronoiGB.setLayout(showL); 

//    QVBoxLayout *robotsL = new QVBoxLayout; 
//    for(int i=0;i<availableRobots.size();i++)
//    {
//    	robotsL->addWidget(availableRobots[i]);
//	    connect(availableRobots[i],        SIGNAL(toggled(bool )), this,SLOT(updateSelectedRobot(bool)));    	
//    }
//    availableRobots[0]->setChecked(true);
//    robotsGB.setLayout(robotsL); 
//	updateRobotSetting();
        
    QVBoxLayout *actionLayout = new QVBoxLayout; 
    actionLayout->addWidget(&pauseBtn); 
    actionLayout->addWidget(&captureImage);     
    actionLayout->addWidget(&pathPlanBtn);
    actionLayout->addWidget(&generateSkeletonBtn); 
    actionLayout->addWidget(&randomTasksBtn); 
    actionGB.setLayout(actionLayout); 

    connect(&distanceToVetix,  SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
//    connect(&selectedRobot,    SIGNAL(itemSelectionChanged()),this, SLOT(handleRobotSelection()));
    connect(&innerSkeletonBtn, SIGNAL(toggled(bool )), this,SLOT(updateSelectedVoronoiMethod(bool)));
    connect(&outerSkeletonBtn, SIGNAL(toggled(bool )), this,SLOT(updateSelectedVoronoiMethod(bool)));    
	connect(&pathPlanBtn,      SIGNAL(pressed()),this, SLOT(pathPlan()));
	connect(&generateSkeletonBtn, SIGNAL(pressed()),tasksGui, SLOT(generateSkeleton()));
	connect(&captureImage,     SIGNAL(pressed()),this, SLOT(save()));	
	connect(&randomTasksBtn,   SIGNAL(pressed()),this, SLOT(pathFollow()));
	connect(&pauseBtn,         SIGNAL(pressed()),this, SLOT(setNavigation()));	
//    connect(&bridgeTest, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setBridgeTest( int ))); 
//    connect(&connectNodes, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setConnNodes( int ))); 
//    connect(&regGrid, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setRegGrid( int ))); 
//    connect(&obstPenalty, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setObstPen( int ))); 
//    connect(&expandObst, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setExpObst( int )));
//    connect(&showTree, SIGNAL(stateChanged(int)),currRobot->planningManager,SLOT(setShowTree( int )));

//	qDebug("Initializing Control Panel --->>>DONE"); fflush(stdout);		
}

void TasksControlPanel::Finished()
{
//	if(currRobot->navigator->isRunning())
//	{
//		currRobot->navigator->StopNavigating();
//		currRobot->navigator->quit();	
//		qDebug("Quitting Thread");
//	}
//	pathFollowBtn.setText("Path Follow");
//	currRobot->notFollowing = true;
}

void TasksControlPanel::pathFollow()
{
//	if(currRobot->notFollowing)
//	{
//		if(currRobot->navigator->isRunning())
//		{
//			currRobot->navigator->quit();
//		}
//		currRobot->navigator->setPath(path);
//		currRobot->navigator->start();
//		pathFollowBtn.setText("Stop");
//		currRobot->notFollowing = false;
//	}
//	else
//	{
//		if(currRobot->navigator->isRunning())
//		{
//			currRobot->navigator->StopNavigating();
//			currRobot->navigator->quit();	
//			qDebug("Quitting Thread");
//		}
//		pathFollowBtn.setText("Path Follow");
//		currRobot->notFollowing = true;
//	}
}

void TasksControlPanel::setNavigation()
{
//	if(currRobot->notPaused)
//	{
//		currRobot->navigator->setPause(true);
//		currRobot->notPaused = false;
//		pauseBtn.setText("Continue");
//	}
//	else
//	{
//		currRobot->navigator->setPause(false);		
//		currRobot->notPaused = true;
//		pauseBtn.setText("Pause");		
//	}
}

void TasksControlPanel::pathPlan()
{
////    if(playGround->renderingMethod == PAINTER_2D)
////	{
////		path = currRobot->planningManager->findPath(PIXEL);
////		navContainer->mapPainter->drawPath(currRobot->planningManager->pathPlanner);
////	}
////	else
//	{
//		path = currRobot->planningManager->findPath(METRIC);						
//	}	
}

void TasksControlPanel::loadMap()
{
//	currRobot->planningManager->setMap(mapPainter->getImage());
//	mapPainter->drawPath(currRobot->planningManager->pathPlanner);
}

void TasksControlPanel::updateSelectedObject(double)
{
//	if(currRobot->planningManager->pathPlanner==NULL)
//	{
//		currRobot->startPlanner();
//	}
//	currRobot->planningManager->setBridgeTestValue(bridgeSegLenSB.value());
//	currRobot->planningManager->setConnNodesValue(nodeConRadSB.value());
//	currRobot->planningManager->setRegGridValue(regGridResSB.value());
//	currRobot->planningManager->setObstPenValue(obstPenRadSB.value());
//	currRobot->planningManager->setExpObstValue(obstExpRadSB.value());
//	currRobot->planningManager->setBridgeResValue(bridgeTestResSB.value());
}

void TasksControlPanel::handleRobotSelection()
{
//    qDebug("Robot Selected"); 
//    QTreeWidgetItem *item = selectedRobot.currentItem();
//	if(widget2RobMan.contains(item))
//	{
//	    currRobot = widget2RobMan.value(item);
//	    qDebug("Robot Name:%s",qPrintable(currRobot->robot->robotName));
//	    fflush(stdout);	    
//	}
//	else
//	{
//	    qDebug("Strange, the selection is not in the list");
//	    currRobot = NULL;
//	    fflush(stdout);	    		
//	}
////  setActionValues(mo); 	
}

void TasksControlPanel::updateSelectedRobot(bool)
{
//	for(int i=0;i<availableRobots.size();i++)
//	{
//		if(availableRobots[i]->isChecked())
//		{
//			currRobot = playGround->robotPlatforms[i];
//			//qDebug("Seleted Robot is:%s",qPrintable(currRobot->robot->robotName));
//			updateRobotSetting();
//			break;
//		}
//	}
}

void TasksControlPanel::updateRobotSetting()
{
//	obstExpRadSB.setValue(currRobot->planningManager->pathPlanner->obstacle_radius);	
//	bridgeTestResSB.setValue(currRobot->planningManager->pathPlanner->bridge_res);
//	bridgeSegLenSB.setValue(currRobot->planningManager->pathPlanner->bridge_length);
//	regGridResSB.setValue(currRobot->planningManager->pathPlanner->reg_grid);	
//	nodeConRadSB.setValue(currRobot->planningManager->pathPlanner->conn_radius);
//	nodeConRadSB.setValue(currRobot->planningManager->pathPlanner->conn_radius);
//	if(navContainer->mapViewer)
//		currRobot->planningManager->setMap(navContainer->mapViewer->image);
//	switch(currRobot->navigator->getObstAvoidAlgo())
//	{
//		case VFH:
//		    vfhRadBtn.setChecked(true);		
//		    break;
//		case FORCE_FIELD:
//		    forceFieldRadBtn.setChecked(true);
//		    break;
//		case CONFIG_SPACE:
//		    configSpaceRadBtn.setChecked(true);
//		    break;
//		case NO_AVOID:
//		    noavoidRadBtn.setChecked(true);		
//		    break;
//		default:
//			qDebug("Unkown ALGO");
//	}
//	if(currRobot->notFollowing)
//	{
//		pathFollowBtn.setText("Path Follow");
//	}
//	else
//	{
//		pathFollowBtn.setText("Stop");		
//	}
//	if(currRobot->notPaused)
//	{
//		pauseBtn.setText("Pause");
//	}
//	else
//	{
//		pauseBtn.setText("Continue");		
//	}
}

void TasksControlPanel::updateSelectedVoronoiMethod(bool)
{
//	if(currRobot->navigator==NULL)
//	{
//		currRobot->startNavigator();
//	}
//	if(vfhRadBtn.isChecked())
//	{
//		qDebug("VFH");
//		currRobot->navigator->setObstAvoidAlgo(VFH);
//	}
//	else if(forceFieldRadBtn.isChecked())
//	{
//		qDebug("Force Field");		
//		currRobot->navigator->setObstAvoidAlgo(FORCE_FIELD);	
//	}
//	else if(configSpaceRadBtn.isChecked())
//	{
//		qDebug("Config Space");		
//		currRobot->navigator->setObstAvoidAlgo(CONFIG_SPACE);	
//	}
//	else if(noavoidRadBtn.isChecked())
//	{
//		qDebug("NO Avoidace");		
//		currRobot->navigator->setObstAvoidAlgo(NO_AVOID);	
//	}
}

void TasksControlPanel::save()
{
//	navContainer->mapViewer->saveImage();
}

void TasksControlPanel::setStart()
{
//	if(this->currRobot)
//	{
//		currRobot->planningManager->setStart(startLoc);
//	}
//	else
//	{
//		qDebug("No Robot is Selected");
//	}
}

void TasksControlPanel::setEnd()
{
//	if(this->currRobot)
//	{
//		currRobot->planningManager->setEnd(endLoc);
//	}
//	else
//	{
//		qDebug("No Robot is Selected");
//	}	
}

void TasksControlPanel::setMap(QImage imageMap)
{
//	if(this->currRobot)
//	{
//		currRobot->planningManager->setMap(imageMap);
//	}
//	else
//	{
//		qDebug("No Robot is Selected");
//	}		
}

void TasksControlPanel::exportHtml()
{
//    QString url=mapManager->exportHtml();
//    QProcess *firefox = new QProcess();
//    QString command = QString("firefox ").append(url);
//    qDebug("Opening %s", qPrintable(command)); 
//    firefox->start(command);
//    firefox->waitForStarted();  
}

MapGL::MapGL():
    QGLWidget(QGLFormat(QGL::AlphaChannel)),
    cursorCentreX(0.53),
    cursorCentreY(0.51), 
    zoomCentreX(0.53), 
    zoomCentreY(0.51), 
    srCentreX(0.54),
    srCentreY(0.55), 
    zoomSizeX(0.08),
    zoomSizeY(0.08), 
    srSizeX(0.54),  
    srSizeY(0.38),
    desiredAspectRatio(1.333)    
{
}

QSize MapGL::setMinimumSizeHint()
{
    //return QSize((int)(240*desiredAspectRatio),240);
    return QSize((int)(600*desiredAspectRatio),600);    
}

QSize MapGL::setSizeHint()
{
    return QSize((int)(600*desiredAspectRatio),600);   
}

void MapGL::initializeGL()
{
//    glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glFlush();
}

void MapGL::setRobotGUI(TasksGui *gui)
{
    tasksGui = gui;
}

void MapGL::resizeGL(int w, int h)
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
        newWidth = (int)(h*desiredAspectRatio); 
        xOffset = (w-newWidth)/2; 
    }
    else if((float) w/h < desiredAspectRatio)
    {
        newHeight= (int)(w/desiredAspectRatio); 
        yOffset = (h-newHeight)/2; 
    }
    qDebug("Resize NEW w = %d h = %d", newWidth, newHeight); 
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity(); 
    glOrtho(0, desiredAspectRatio, 0, 1, -200, 200); 
    //glOrtho(-1, 1, -1, 1, 0, 0); 
    glMatrixMode(GL_MODELVIEW);
    glViewport( xOffset, yOffset, (GLint)newWidth, (GLint)newHeight );
}

void MapGL::config()
{
    //laserEnabled = true;//(bool) cf->ReadInt(sectionid, "laserEnabled", 1); 
    //speedEnabled = true;//(bool) cf->ReadInt(sectionid, "speedEnabled", 1);
    //mapEnabled   = true;//(bool) cf->ReadInt(sectionid, "mapEnabled", 1);
//    if(speedEnabled)
//    {
//		speedMeter.setMaxSpeed(2); 
//		speedMeter.setMaxTurnRate(2);
//    }
}
void MapGL::setSSkelPtr(SSkelPtr sskel)
{
	this->sskel = sskel;
}
void MapGL::renderSkeleton()
{
    SSkel::Traits::Construct_segment_2 construct_segment ;
  	const Halfedge_const_handle null_halfedge ;
  	const Vertex_const_handle   null_vertex ;
  	
    if ( !sskel )
      return ;

    int watchdog_limit = sskel->size_of_halfedges();
	
	glPushMatrix();
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POLYGON_SMOOTH); 
    glEnable(GL_BLEND);
//    glTranslatef(camPanelRatio*0.75, 0.75,0);
//    glScalef(camPanelRatio*0.2,0.2,1); 
	glColor4f(0,0,0,1);
	glLineWidth(2);
    	glBegin(GL_LINE_STRIP); 
    	    
    for ( Face_const_iterator fit = sskel->faces_begin(), efit = sskel->faces_end()
          ; fit != efit
          ; ++ fit
        )
    {
      Halfedge_const_handle hstart = fit->halfedge();
      Halfedge_const_handle he     = hstart ;
      
      int watchdog = watchdog_limit ;   

      do
      {
        if ( he == null_halfedge )
          break ;

        if ( he->is_bisector() )
        {
          bool lVertexOK      = he->vertex() != null_vertex ;
          bool lOppositeOK    = he->opposite() != null_halfedge ;
          bool lOppVertexOK   = lOppositeOK && he->opposite()->vertex() != null_vertex ;
          bool lVertexHeOK    = lVertexOK && he->vertex()->halfedge() != null_halfedge ;
          bool lOppVertexHeOK = lOppVertexOK && he->opposite()->vertex()->halfedge() != null_halfedge ;

          if ( lVertexOK && lOppVertexOK && lVertexHeOK && lOppVertexHeOK )
          {

		    	glVertex2f(he->opposite()->vertex()->point().x(),he->opposite()->vertex()->point().y());
		    	glVertex2f(he->vertex()->point().x(),he->vertex()->point().y());
//            *widget << ( he->is_inner_bisector()? CGAL::BLUE : CGAL::GREEN ) ;
//            *widget << construct_segment(he->opposite()->vertex()->point(),he->vertex()->point()) ;
          }
        }
        he = he->next();
      }
      while ( -- watchdog > 0 && he != hstart ) ;

    }
	glEnd();
    glDisable(GL_BLEND);
    glDisable(GL_POLYGON_SMOOTH);
    glPopMatrix();           	
}

void MapGL::paintGL()
{
//    double camPanelRatio = 1.0; 
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
//     if(laserEnabled)
//     {
//         glPushMatrix(); 
//         glTranslatef(camPanelRatio*.7,0.25, 0);
//         glScalef(0.2,0.2,1);
//         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//         glEnable(GL_POLYGON_SMOOTH); 
//         glEnable(GL_BLEND);
//         laser.render();
//         glDisable(GL_BLEND);
//         glDisable(GL_POLYGON_SMOOTH);
//         glPopMatrix(); 
//     }    
//     if(speedEnabled)
//     {
//         glPushMatrix();
//         glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//         glEnable(GL_POLYGON_SMOOTH); 
//         glEnable(GL_BLEND);
//         glTranslatef(camPanelRatio*0.75, 0.75,0);
//         glScalef(camPanelRatio*0.2,0.2,1); 
//         speedMeter.render(); 
//         glDisable(GL_BLEND);
//         glDisable(GL_POLYGON_SMOOTH);
//         glPopMatrix(); 
//     }
//     if(mapEnabled)
//     {
//	   //  mapViewer.render();
//     }     
	renderSkeleton();
	glFlush();
}

TasksGui::TasksGui(QWidget *parent): 
	   tabContainer((QTabWidget*) parent),
       speed(0.15), 
	   turnRatio(5),
	   ptzPan(0),
	   ptzTilt(0),
	   radPerPixel(0.001),
	   msperWheel(0.0005),
	   tasksControlPanel(this,parent)
{
    
    QHBoxLayout *layout = new QHBoxLayout();
	layout->addWidget(&mapGL,1);
	layout->addWidget(&tasksControlPanel,1);	
    setLayout(layout); 
    updateGeometry();
    setFocusPolicy(Qt::StrongFocus);
    config();
}

void TasksGui::updateData()
{
    
}

int TasksGui::config()
{
    QString commsName ="Wheelchair";
    mapGL.setRobotGUI(this); 
    mapGL.config();
    // signals for changing modes
    //connect( OGRadBtn, SIGNAL(clicked()), this, SLOT(renderOG()));
    //connect( laserRadBtn, SIGNAL(clicked()), this, SLOT(renderLaser()));
    //connect( staticRadBtn, SIGNAL(clicked()), this, SLOT(renderStatic()));
    // robotManager->commManager->start();
    return 1; 

}

void TasksGui::mousePressEvent(QMouseEvent *me)
{
    qDebug("Mouse pressed"); 
    startX = me->x();
    startY = me->y();
}

void TasksGui::mouseReleaseEvent(QMouseEvent *me)
{
    qDebug("Mouse Released"); 
    startX = me->x();
    startY = me->y();
}

void TasksGui::provideSpeed( double &in_speed, double &in_turnRate)
{
    in_speed = speed; 
    in_turnRate = turnRatio * speed;
}

void TasksGui::mouseMoveEvent(QMouseEvent *me)
{
    startX = me->x(); 
    startY = me->y(); 
    //mapGL.moveDrivePan(-0.002*deltaX);  
    mapGL.updateGL();
    if(ptzEnabled)
    {
		int deltaX = int(me->x() - startX); 
		int deltaY = int(me->y() - startY); 
		ptzPan -= deltaX*radPerPixel; 
		ptzTilt += deltaY*radPerPixel; 
	    //qDebug("Intermediate arm pos: %f %f %f", currentArmPos[0], currentArmPos[1], currentArmPos[2]);
		startX = me->x(); 
		startY = me->y(); 
    }
}


void TasksGui::keyPressEvent(QKeyEvent *e)
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
//                robotManager->commManager->setSpeed(speed); 
                break;
            case 'a':
//                robotManager->commManager->setTurnRate(turnRatio*speed); 
                break;
            case 's':
//                robotManager->commManager->setSpeed(-speed);
                break;
            case 'd':
//                robotManager->commManager->setTurnRate(-turnRatio*speed); 
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
//                if(isFullScreen()){
//                    fullScreenOff(); 
//                }
//                else {
//                    fullScreenOn();
//                }
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

void TasksGui::keyReleaseEvent(QKeyEvent *e)
{
    if(!e->isAutoRepeat())
    {
        // Stop 
        switch(e->text().at(0).toAscii())
        {
        case 'w': 
        case 's':
//            robotManager->commManager->setSpeed(0); 
            break;
        case 'a':
        case 'd':
//            robotManager->commManager->setTurnRate(0); 
            break;
        }
    }
}
 
void TasksGui::wheelEvent( QWheelEvent *we)
{
    // check for mouse move direction
    int deltaMoved = we->delta();
    if( deltaMoved < 0 )
        speed -= 0.01; // decrease Homer speed a bit
    else
        speed += 0.01; // increase speed
}

TasksGui::~TasksGui()
{
}

void TasksGui::renderOG()
{
    //resetTab();
	setRadMode(0);
}

void TasksGui::renderLaser()
{
    //resetTab();
	setRadMode(1);
}
void TasksGui::renderStatic()
{
    //resetTab();
	setRadMode(2);
}

void TasksGui::setRadMode(int mode)
{
	qDebug("Radio Button Clicked");
    switch(mode)
    {
        case 0: // AUTO_TELEOP
//        	mapGL.mapEnabled   = true;
//        	mapGL.laserEnabled = false;
//        	mapGL.speedEnabled = false;
//            OGRadBtn->setChecked(true);
            qDebug("OG MAP enabled");
            break;
        case 1: // AUTO_PAUSED
//            mapGL.mapEnabled   = false;
//        	mapGL.laserEnabled = true;
//        	mapGL.speedEnabled = false;
//            laserRadBtn->setChecked(true);
            qDebug("Laser Enabled");
            break;
        case 2: // AUTO_FULL
//           	mapGL.mapEnabled   = false;
//        	mapGL.laserEnabled = false;
//        	mapGL.speedEnabled = true;
//            staticRadBtn->setChecked(true);
            qDebug("Speed Enabled");            
            break;
        default:
	    qDebug("Mode is incorrect");
    }
}

void TasksGui::resetTab()
{
    // get this tab index
    int index = tabContainer->indexOf(this);

    // set tab title
    tabContainer->setTabText(index, "Cas Planner");

    // set Victim Signal icon
    //tabContainer->setTabIcon(index, QIcon("../ui/rescuegui-branchCommsRefactor/blank.xpm"));
}

void TasksGui::requestSnap()
{

}

void TasksGui::generateSkeleton()
{
	qDebug("Generating Skeleton");
	mapSkeleton.loadMap();
	mapSkeleton.generateInnerSkeleton();
	qDebug("Skeleton Done");
	mapGL.setSSkelPtr(mapSkeleton.getSSkelPtr());
}

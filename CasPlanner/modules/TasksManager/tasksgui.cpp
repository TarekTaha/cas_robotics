#include "tasksgui.h"

TasksControlPanel::TasksControlPanel(TasksGui *tasksGui,QWidget *parent):
	QWidget(parent),
	tasksGui(tasksGui),
	randomTasksGB("Random Tasks"),
	numRandomRuns(),
	voronoiGB("Map Voronoi Skeleton"),
	innerSkeletonBtn("Inner Skeleton"),
	outerSkeletonBtn("Outer Skeleton"),
	actionGB("Action"),
	pauseBtn("Pause"),
	randomTasksBtn("Run Random Tasks"),
	generateSkeletonBtn("Generate Skeleton"),
	captureImage("Capture Image"),
	tasksGB("Set of Tasks"),
	tasksList(this)
{
    QVBoxLayout *hlayout = new QVBoxLayout;

    hlayout->addWidget(&tasksGB,1);
    hlayout->addWidget(&randomTasksGB,1);
    hlayout->addWidget(&voronoiGB,1);
    hlayout->addWidget(&actionGB,1);
    this->setLayout(hlayout);


	QVBoxLayout *tasksLayout = new QVBoxLayout;    
	tasksLayout->addWidget(&tasksList);
    tasksLayout->addWidget(&numRandomRuns);	
	tasksGB.setLayout(tasksLayout);
	
	QHBoxLayout *parHLayout = new QHBoxLayout; 
	QVBoxLayout *parVLayout = new QVBoxLayout;	
    parHLayout->addWidget(new QLabel("Random Runs"));
    parHLayout->addWidget(&numRandomRuns);
	
	parVLayout->addLayout(parHLayout);
    parVLayout->addWidget(&randomTasksBtn);    
    randomTasksGB.setLayout(parVLayout);

    numRandomRuns.setMinimum(0);
    numRandomRuns.setMaximum(100);
	numRandomRuns.setSingleStep(1);
	numRandomRuns.setValue(10);

    QVBoxLayout *showL = new QVBoxLayout;
    showL->addWidget(&innerSkeletonBtn);
    showL->addWidget(&outerSkeletonBtn);
    innerSkeletonBtn.setChecked(true);
	updateSelectedVoronoiMethod(true);
    voronoiGB.setLayout(showL);
	
    QVBoxLayout *actionLayout = new QVBoxLayout;
    actionLayout->addWidget(&pauseBtn);
    actionLayout->addWidget(&captureImage);
    actionLayout->addWidget(&generateSkeletonBtn);
    actionGB.setLayout(actionLayout);

//	connect(&numRandomRuns,  SIGNAL(valueChanged(double)), this, SLOT(updateSelectedObject(double)));
    connect(&innerSkeletonBtn, SIGNAL(toggled(bool )), this,SLOT(updateSelectedVoronoiMethod(bool)));
  	connect(&outerSkeletonBtn, SIGNAL(toggled(bool )), this,SLOT(updateSelectedVoronoiMethod(bool)));
	connect(&generateSkeletonBtn, SIGNAL(pressed()),tasksGui, SLOT(generateSkeleton()));
	connect(&captureImage,     SIGNAL(pressed()),this, SLOT(save()));
	connect(&randomTasksBtn,   SIGNAL(pressed()),this, SLOT(runRandomTasks()));
	connect(&tasksList,        SIGNAL(currentRowChanged(int)),this, SLOT(taskSelected(int)));
//	connect(&tasksList,        SIGNAL(itemClicked(QListWidgetItem * item )),this, SLOT(taskClicked(QListWidgetItem * item)));	
//	connect(&pauseBtn,         SIGNAL(pressed()),this, SLOT(setNavigation()));
}


void TasksControlPanel::loadMap()
{

}

void TasksControlPanel::runRandomTasks()
{
	QTime t;
	Node * p;
	if(!tasksGui->skeletonGenerated)
		tasksGui->generateSkeleton();
	int r;
	t.start();
	for(int i=0;i<numRandomRuns.value();i++)
	{
		r = rand()%tasksGui->tasks.size();
		//tasksList.setCurrentRow(r);
		qDebug("Using Task %s %d ",qPrintable(tasksGui->tasks[r].getName()),r);
		Pose start(tasksGui->tasks[r].getStart().x(),tasksGui->tasks[r].getStart().y(),0);
		Pose   end(tasksGui->tasks[r].getEnd().x(),tasksGui->tasks[r].getEnd().y(),0);		
		tasksGui->voronoiPlanner->startSearch(start,end,METRIC);
		if(tasksGui->voronoiPlanner->path)
		{
			tasksGui->voronoiPlanner->printNodeList();				
			p = tasksGui->voronoiPlanner->path;
			while(p)
			{
				for(int j=0; j<tasksGui->playGround->mapManager->mapSkeleton.verticies.size(); j++)
				{
					if((p->pose.p.x() == tasksGui->playGround->mapManager->mapSkeleton.verticies[j].location.x())&&
					   (p->pose.p.y() == tasksGui->playGround->mapManager->mapSkeleton.verticies[j].location.y()))
					   {
					   		tasksGui->playGround->mapManager->mapSkeleton.verticies[j].prob = (++tasksGui->playGround->mapManager->mapSkeleton.verticies[j].visits)/double(++tasksGui->totalVisits); 
					   }
				}		
				p = p->next;
			}
		}	
	}
	qDebug("Generationg %f Random Paths took:%d",numRandomRuns.value(),t.elapsed());
}

void TasksControlPanel::updateSelectedVoronoiMethod(bool)
{

}

void TasksControlPanel::save()
{
//	navContainer->mapViewer->saveImage();
}

void TasksControlPanel::taskSelected(int r)
{
	qDebug("Selected Task is:%d",r);	
	Pose start(tasksGui->tasks[r].getStart().x(),tasksGui->tasks[r].getStart().y(),0);
	Pose   end(tasksGui->tasks[r].getEnd().x(),tasksGui->tasks[r].getEnd().y(),0);
	if(tasksGui->voronoiPlanner)		
		tasksGui->voronoiPlanner->startSearch(start,end,METRIC);	
	fflush(stdout);
}

void TasksControlPanel::setMap(QImage)
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

MapGL::MapGL(TasksGui *tsg, QWidget *parent):
	QGLWidget(QGLFormat(QGL::AlphaChannel), parent),
	//sskel(ssk),
	tasksGui(tsg),
	zoomFactor(10),
 	xOffset(0),
 	yOffset(0),
 	zOffset(0),
 	yaw(0),
 	pitch(0),
 	fudgeFactor(3),
	showGrids(true),
	firstTime(true)
{
	renderTimer = new QTimer(this);
	connect(renderTimer, SIGNAL(timeout()), this, SLOT(updateGL()));
	renderTimer->start(100);
	setFocusPolicy(Qt::StrongFocus);
}

QSize MapGL::sizeHint()
{
	//return QSize(800,600);
	return QSize(800,800);
}

QSize MapGL::setMinimumSizeHint()
{
    //return QSize(400,300);
	return QSize(400,400);    
}

void MapGL::initializeGL()
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glFlush();
    skeletonList = glGenLists(1);
}

void MapGL::resizeGL(int w, int h)
{
	aspectRatio = ((float) w)/((float) h);
	// Reset the coordinate system before modifying
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(180, aspectRatio, 1,1000);
//    qDebug("Resize NEW w = %d h = %d", w, h);
	glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glOrtho(-aspectRatio, aspectRatio, -1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	updateGL();
}

void MapGL::config()
{

}

void MapGL::setSSkelPtr(SSkelPtr sskel)
{
 	this->sskel = sskel;
}

void MapGL::drawProbHisto(QPointF pos, double prob)
{
	QString str = QString("%1 \%").arg((int)(prob*100));  
	if(prob==0)
		return;
	glPushMatrix();
	glTranslatef(pos.x(),pos.y(),0.0f);
    glColor4f(0,0,0,1);
 	renderText(0 ,0 + 0.2, prob + 0.2, str);
	glScalef(1/12.0, 1/12.0, prob);
  	//glRotatef(rotqube,0.0f,1.0f,0.0f);	// Rotate The cube around the Y axis
  	//glRotatef(rotqube,1.0f,1.0f,1.0f);
  	glBegin(GL_QUADS);		// Draw The Cube Using quads  
	
 		//glColor3f(1.0f,0.0f,1.0f);	// Color Violet
		glColor4f(0.0f,1.0f,0.0f,1.0f); 		
	    
//	    glColor3f(0.0f,1.0f,0.0f);	// Color Blue
	    glVertex3f( 1.0f, 1.0f,-0.0f);	// Top Right Of The Quad (Top)
	    glVertex3f(-1.0f, 1.0f,-0.0f);	// Top Left Of The Quad (Top)
	    glVertex3f(-1.0f, 1.0f, 1.0f);	// Bottom Left Of The Quad (Top)
	    glVertex3f( 1.0f, 1.0f, 1.0f);	// Bottom Right Of The Quad (Top)
	    
//	    glColor3f(1.0f,0.5f,0.0f);	// Color Orange
	    glVertex3f( 1.0f,-1.0f, 1.0f);	// Top Right Of The Quad (Bottom)
	    glVertex3f(-1.0f,-1.0f, 1.0f);	// Top Left Of The Quad (Bottom)
	    glVertex3f(-1.0f,-1.0f,-0.0f);	// Bottom Left Of The Quad (Bottom)
	    glVertex3f( 1.0f,-1.0f,-0.0f);	// Bottom Right Of The Quad (Bottom)
	    
//	    glColor3f(1.0f,0.0f,0.0f);	// Color Red	
	    glVertex3f( 1.0f, 1.0f, 1.0f);	// Top Right Of The Quad (Front)
	    glVertex3f(-1.0f, 1.0f, 1.0f);	// Top Left Of The Quad (Front)
	    glVertex3f(-1.0f,-1.0f, 1.0f);	// Bottom Left Of The Quad (Front)
	    glVertex3f( 1.0f,-1.0f, 1.0f);	// Bottom Right Of The Quad (Front)
	    
//	    glColor3f(1.0f,1.0f,0.0f);	// Color Yellow
	    glVertex3f( 1.0f,-1.0f,-0.0f);	// Top Right Of The Quad (Back)
	    glVertex3f(-1.0f,-1.0f,-0.0f);	// Top Left Of The Quad (Back)
	    glVertex3f(-1.0f, 1.0f,-0.0f);	// Bottom Left Of The Quad (Back)
	    glVertex3f( 1.0f, 1.0f,-0.0f);	// Bottom Right Of The Quad (Back)
	    
//	    glColor3f(0.0f,0.0f,1.0f);	// Color Blue
	    glVertex3f(-1.0f, 1.0f, 1.0f);	// Top Right Of The Quad (Left)
	    glVertex3f(-1.0f, 1.0f,-0.0f);	// Top Left Of The Quad (Left)
	    glVertex3f(-1.0f,-1.0f,-0.0f);	// Bottom Left Of The Quad (Left)
	    glVertex3f(-1.0f,-1.0f, 1.0f);	// Bottom Right Of The Quad (Left)
	    
//	    glColor3f(1.0f,0.0f,1.0f);	// Color Violet
	    glVertex3f( 1.0f, 1.0f,-0.0f);	// Top Right Of The Quad (Right)
	    glVertex3f( 1.0f, 1.0f, 1.0f);	// Top Left Of The Quad (Right)
	    glVertex3f( 1.0f,-1.0f, 1.0f);	// Bottom Left Of The Quad (Right)
	    glVertex3f( 1.0f,-1.0f,-0.0f);	// Bottom Right Of The Quad (Right)
  	glEnd();			// End Drawing The Cube
	glPopMatrix();  	
	return;
}

void MapGL::renderSkeleton()
{
  	const Halfedge_const_handle null_halfedge ;
  	const Vertex_const_handle   null_vertex ;
  		
    if ( !this->sskel )
      return ;

	fflush(stdout);
    int watchdog_limit = sskel->size_of_halfedges();

	glPushMatrix();
	//glLineWidth(2);
	for ( Face_const_iterator fit = sskel->faces_begin(), efit = sskel->faces_end(); fit != efit; ++ fit)
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
			    	he->is_inner_bisector()? glColor4f(0,0,1,1) : glColor4f(1,0,0,1);
					glBegin(GL_LINES);
						//if(he->opposite()->vertex()->is_skeleton())
						glVertex2f(he->opposite()->vertex()->point().x(),he->opposite()->vertex()->point().y());
		    			glVertex2f(he->vertex()->point().x(),he->vertex()->point().y());
					glEnd();
//					if (firstTime)
//					{
//						std::cout<<"\nDrawing Line at Start X:"<<he->opposite()->vertex()->point().x()<<" Y:"<<he->opposite()->vertex()->point().y();
//						std::cout<<" End X:"<<he->vertex()->point().x()<<" Y:"<<he->vertex()->point().y();
//					}
//					if(he->vertex()->is_skeleton())
//					{
//						glBegin(GL_POLYGON);
//							glVertex2f(he->vertex()->point().x()- 0.1, he->vertex()->point().y()+0.1);
//							glVertex2f(he->vertex()->point().x()+ 0.1, he->vertex()->point().y()+0.1);
//							glVertex2f(he->vertex()->point().x()+ 0.1, he->vertex()->point().y()-0.1);
//							glVertex2f(he->vertex()->point().x()- 0.1, he->vertex()->point().y()-0.1);
//						glEnd();
//					}
//					if(he->opposite()->vertex()->is_skeleton())
//					{
//						glBegin(GL_POLYGON);
//							glVertex2f(he->opposite()->vertex()->point().x()- 0.1, he->opposite()->vertex()->point().y()+0.1);
//							glVertex2f(he->opposite()->vertex()->point().x()+ 0.1, he->opposite()->vertex()->point().y()+0.1);
//							glVertex2f(he->opposite()->vertex()->point().x()+ 0.1, he->opposite()->vertex()->point().y()-0.1);
//							glVertex2f(he->opposite()->vertex()->point().x()- 0.1, he->opposite()->vertex()->point().y()-0.1);
//						glEnd();
//					}				
          		}	
        	}
        	he = he->next();
      	}
      	while ( -- watchdog > 0 && he != hstart ) ;
    }
    for(int i=0;i<tasksGui->playGround->mapManager->mapSkeleton.verticies.size();i++)
    {
		drawProbHisto(tasksGui->playGround->mapManager->mapSkeleton.verticies[i].location,tasksGui->playGround->mapManager->mapSkeleton.verticies[i].prob);    	
    }
	firstTime = false;
    glPopMatrix();
}

void MapGL::renderPath()
{
	if(!tasksGui->voronoiPlanner)
	{
		//qDebug("WHAT THEEEE !!!");
		return;
	}
	if(tasksGui->voronoiPlanner->path)
	{
		Node * path = tasksGui->voronoiPlanner->path;
		//glColor4f(1,1,1,1);
		glPushMatrix();
		glColor4f(0,0,1,1);
		glLineWidth(5);
	    glBegin(GL_LINE_STRIP);
		while(path)
		{
	    	glVertex2f(path->pose.p.x(),path->pose.p.y());
			path = path->next;
		}
	    glEnd();
	    glLineWidth(1);
		glPopMatrix();	    
	}
	//qDebug("WHAT THEEEE !!!");	
}

void MapGL::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	//glEnable(GL_DEPTH_TEST);
	//glDisable(GL_BLEND);
	//glEnable(GL_POINT_SMOOTH);
	//glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	//glEnable(GL_LINE_SMOOTH);
	//glEnable(GL_POLYGON_SMOOTH);

	glPushMatrix();
	glScalef(1/zoomFactor, 1/zoomFactor, 1/zoomFactor);
    glColor4f(0,0,0,1);
 	renderText(zoomFactor*aspectRatio*0.90-1, -0.9*zoomFactor, 0, "grid: 1 m");

	glRotatef(pitch,1,0,0);
	glRotatef(yaw,0,0,1);

	glTranslatef(xOffset, yOffset, zOffset);
	glPushMatrix();
	showGrids = true;
	//glLineWidth(1);
	if(showGrids)
	{
		for(int i=-(int) zoomFactor*3; i < (int) zoomFactor*3; i++)
		{
			glBegin(GL_LINES);
			if(i==0)
			{
				glColor4f(0,0,0,0.5);
			}
			else
			{
				glColor4f(0.5,0.5,0.5,0.5);
			}
			glVertex3f(-zoomFactor*3, i, 0);
			glVertex3f(zoomFactor*3, i, 0);
			glVertex3f(i,-zoomFactor*3, 0);
			glVertex3f(i, zoomFactor*3, 0);
			glEnd();
		}
// 		// X-axis indicator
// 		int i = int((mapData->width*mapData->resolution)/2.0 + 2);
// 		{
// 			glBegin(GL_LINE_LOOP);
// 			glColor4f(0,0,0,0.5);
// //				glColor4f(1,1,1,0.5);
// 			glVertex3f(i-1,0.5,0);
// 			glVertex3f(i,0,0);
// 			glVertex3f(i-1,-0.5,0);
// 			glEnd();
// 			renderText(i,-1,0, "X");
// 		}
// 		 //Y-axis indicator
// 		int j = int((mapData->height*mapData->resolution)/2.0 + 2);
// 		{
// 			glBegin(GL_LINE_LOOP);
// 			glColor4f(0,0,0,0.5);
// //				glColor4f(1,1,1,0.5);
// 			glVertex3f(-0.5,j-1,0);
// 			glVertex3f(0,j,0);
// 			glVertex3f(0.5,j-1,0);
// 			glEnd();
// 			renderText(1,j,0, "Y");
// 		}
	}
	glPopMatrix();
	renderSkeleton();
	renderPath();
	if(tasksGui->skeletonGenerated && firstTime)
	{
		glNewList(skeletonList, GL_COMPILE);	
			tasksGui->cvd->draw_diagram();
			tasksGui->cvd->draw_sites();
		glEndList();
		glCallList(skeletonList);
		firstTime = false;
	}
	else
		glCallList(skeletonList);
	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_POINT_SMOOTH);
	glDisable(GL_LINE_SMOOTH);
	glDisable(GL_POLYGON_SMOOTH);
	glPopMatrix();
	glFlush();
}

void MapGL::keyPressEvent(QKeyEvent *e)
{
	if(e->key() == Qt::Key_C)
	{
		if(e->modifiers() && Qt::ShiftModifier)
		{
		}
		else
		{
		}
	}
	else if(e->key() == Qt::Key_W)
	{
		if(e->modifiers() && Qt::ShiftModifier)
		{

		}
		else
		{
			yOffset += 0.1*zoomFactor;
		}
	}
	else if(e->key() == Qt::Key_S)
	{
		if(e->modifiers() && Qt::ShiftModifier)
		{

		}
		else
		{
			yOffset -= 0.1*zoomFactor;
		}
	}
	else if(e->key() == Qt::Key_A)
	{
		if(e->modifiers() && Qt::ShiftModifier)
		{

		}
		else
		{
			xOffset -= 0.1*zoomFactor;
		}
	}
	else if(e->key() == Qt::Key_D)
	{
		if(e->modifiers() && Qt::ShiftModifier)
		{

		}
		else
		{
			xOffset += 0.1*zoomFactor;
		}
	}
	else if(e->key() == Qt::Key_BracketLeft)
	{
		zoomFactor *= 1.1;
		qDebug("ZoomFactor set to %f", zoomFactor);
	}
	else if(e->key() == Qt::Key_BracketRight)
	{
		zoomFactor /= 1.1;
		qDebug("ZoomFactor set to %f", zoomFactor);
	}
	else if(e->key() == Qt::Key_Left)
	{
		if(e->modifiers() && Qt::ShiftModifier)
		{

		}
		else
		{
			yaw += 5;
		}
	}
	else if(e->key() == Qt::Key_Right)
	{
		if(e->modifiers() && Qt::ShiftModifier)
		{

		}
		else
		{
			yaw -= 5;
		}
	}
	else if(e->key() == Qt::Key_Up)
	{
		pitch += 5;
		if(pitch > 90) pitch = 90;
	}
	else if(e->key() == Qt::Key_Down)
	{
		pitch -= 5;
		if(pitch < -90) pitch = -90;
	}
	else if(e->key() == Qt::Key_R)
	{
		zoomFactor=10;
		xOffset= yOffset=zOffset=yaw=pitch=0;
	}
	else if(e->text() == "=")
	{
		fudgeFactor *=1.25;
		qDebug("Fudge factor set to %f", fudgeFactor);
	}
	else if(e->text()=="-")
	{
		fudgeFactor /=1.25;
		qDebug("Fudge factor set to %f", fudgeFactor);
	}
	else if(e->text() == "0")
	{
		fudgeFactor=3;
		qDebug("Fudge factor set to %f", fudgeFactor);
	}
	updateGL();
}

TasksGui::TasksGui(QWidget *parent,PlayGround *playG):
	voronoiPlanner(NULL),
	skeletonGenerated(false),
	totalVisits(0),
	playGround(playG),
	tabContainer((QTabWidget*) parent),
	tasksControlPanel(this,parent),
	mapGL(this,parent),	
    speed(0.15),
	turnRatio(5),
	ptzPan(0),
	ptzTilt(0),
	radPerPixel(0.001),
	msperWheel(0.0005)
{
    QHBoxLayout *layout = new QHBoxLayout();
	layout->addWidget(&mapGL,4);
	layout->addWidget(&tasksControlPanel,1);
    setLayout(layout);
    updateGeometry();
    setFocusPolicy(Qt::StrongFocus);
    config();
	sskel = defs::SSkelPtr();
	loadTasks("/home/BlackCoder/workspace/CasPlanner/modules/TasksManager/tasks.txt");	
}

void TasksGui::updateData()
{

}

int TasksGui::config()
{
    QString commsName ="Wheelchair";
    //mapGL.config();
    // signals for changing modes
    //connect( OGRadBtn, SIGNAL(clicked()), this, SLOT(renderOG()));
    //connect( laserRadBtn, SIGNAL(clicked()), this, SLOT(renderLaser()));
    //connect( staticRadBtn, SIGNAL(clicked()), this, SLOT(renderStatic()));
    // robotManager->commManager->start();
    return 1;

}

void TasksGui::loadTasks(string filename)
{
 	std::ifstream in(filename.c_str());
 	tasks.clear();
 	int row=0;
	if ( in )
    {
    	while ( ! in.eof() )
      	{ 
          	double x1,x2,y1,y2 ;
          	string name;
          	//char name[20];
          	in >> x1 >> y1 >> x2 >> y2>> name ;
          	Task t(QPointF(x1,y1),QPointF(x2,y2),name.c_str());
	        tasks.push_back(t);
	        
	        QListWidgetItem *newItem = new QListWidgetItem;
	        newItem->setText(name.c_str());
	        tasksControlPanel.tasksList.insertItem(row++, newItem);          	 
      	}
    }
	else
	{
		std::cout<<"\n File Not Found";
	}    
//	for(int i=0; i<tasks.size();i++)
//	{
//		qDebug("Task %d: from [%f %f] to [%f %f] Name:%s ",i+1,tasks[i].getStart().x(),tasks[i].getStart().y(),
//		tasks[i].getEnd().x(),tasks[i].getEnd().y(),qPrintable(tasks[i].getName()));
//	}
}
// void TasksGui::mousePressEvent(QMouseEvent *me)
// {
//     qDebug("Mouse pressed");
//     startX = me->x();
//     startY = me->y();
// }
//
// void TasksGui::mouseReleaseEvent(QMouseEvent *me)
// {
//     qDebug("Mouse Released");
//     startX = me->x();
//     startY = me->y();
// }

void TasksGui::provideSpeed( double &in_speed, double &in_turnRate)
{
    in_speed = speed;
    in_turnRate = turnRatio * speed;
}

// void TasksGui::mouseMoveEvent(QMouseEvent *me)
// {
//     startX = me->x();
//     startY = me->y();
//     //mapGL.moveDrivePan(-0.002*deltaX);
//     mapGL.updateGL();
//     if(ptzEnabled)
//     {
// 		int deltaX = int(me->x() - startX);
// 		int deltaY = int(me->y() - startY);
// 		ptzPan -= deltaX*radPerPixel;
// 		ptzTilt += deltaY*radPerPixel;
// 	    //qDebug("Intermediate arm pos: %f %f %f", currentArmPos[0], currentArmPos[1], currentArmPos[2]);
// 		startX = me->x();
// 		startY = me->y();
//     }
// }


// void TasksGui::keyPressEvent(QKeyEvent *e)
// {
//     QString text;
//     bool ok;
//
//     //qDebug("Key pressed %s", qPrintable(e->text()));
//         if(e->key() == Qt::Key_Up)
//         {
//             qDebug("Cursor moved up");
//             //emit cursorMove(0,1);
//         }
//         else if(e->key() == Qt::Key_Down)
//         {
//             //emit cursorMove(0,-1);
//         }
//         else if(e->key() == Qt::Key_Left)
//         {
//             //emit cursorMove(-1,0);
//         }
//         else if(e->key() == Qt::Key_Right)
//         {
//             //emit cursorMove(1,0);
//         }
//         switch(e->text().at(0).toAscii())
//         {
//             case 'w':
// //                robotManager->commManager->setSpeed(speed);
//                 break;
//             case 'a':
// //                robotManager->commManager->setTurnRate(turnRatio*speed);
//                 break;
//             case 's':
// //                robotManager->commManager->setSpeed(-speed);
//                 break;
//             case 'd':
// //                robotManager->commManager->setTurnRate(-turnRatio*speed);
//                 break;
//             case 'z':
//
//             case '.':
//                 //emit cursorCentre();
//                 break;
// 	    case 'n':
// 		// Start new patch
// 		//robotManager->commManager->requestNewPatch(0);
// 		break;
//             case 'v':
// 	       		text = QInputDialog::getText(this, "Name Dialog", "Please Enter your name: ",
//                	QLineEdit::Normal, QString::null, &ok);
// 				if ( ok )
// 				{
// 		    		if(text.isEmpty())
// 		    		{
//     					text = "NoName";
// 		    		}
// 			        int index = tabContainer->indexOf(this);
// 			        tabContainer->setTabText(index, "CAS PLANNER");
// 				}
//                 break;
//             case 'l':
//                 //
//                 break;
//             case 'c':
//                 //emit newMode(linkmode);
//                 break;
//             case 'm':
//                 //mouseInvert = (-1)*mouseInvert;
//                 break;
//             case 'I':
//                 //emit cancelMode();
//                 break;
//             case 'f':
// //                if(isFullScreen()){
// //                    fullScreenOff();
// //                }
// //                else {
// //                    fullScreenOn();
// //                }
//                 break;
//             case 'h':
//                 //hideHUD = !hideHUD;
//                 break;
//             case 'x':
//                 //hideExtras = !hideExtras;
//                 break;
//             case  '\'':
//                 break;
//             case '/':
//                 break;
//             case '1':
//             case '2':
//             case '3':
//             case '4':
//             case '5':
//             case '6':
//             case '7':
//             case '8':
//             case '9':
//             case '0':
//                 break;
//             case 'b':
//                 break;
//             default:
//                 e->ignore();
//         }
//
// }

// void TasksGui::keyReleaseEvent(QKeyEvent *e)
// {
//     if(!e->isAutoRepeat())
//     {
//         // Stop
//         switch(e->text().at(0).toAscii())
//         {
//         case 'w':
//         case 's':
// //            robotManager->commManager->setSpeed(0);
//             break;
//         case 'a':
//         case 'd':
// //            robotManager->commManager->setTurnRate(0);
//             break;
//         }
//     }
// }

// void TasksGui::wheelEvent( QWheelEvent *we)
// {
//     // check for mouse move direction
//     int deltaMoved = we->delta();
//     if( deltaMoved < 0 )
//         speed -= 0.01; // decrease Homer speed a bit
//     else
//         speed += 0.01; // increase speed
// }

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

    //tabContainer->setTabIcon(index, QIcon("../ui/rescuegui-branchCommsRefactor/blank.xpm"));
}

void TasksGui::requestSnap()
{

}

//void TasksGui::generateSkeleton()
//{
//    cvd = new VoronoiDiagram();
//    vvd = cvd;	
//    vvd->clear();
//    int counter=0;
//    if(playGround && playGround->mapManager)
//    {
//    	Map * map = playGround->mapManager->globalMap;
//    	for(int i=0;i<map->width;i++)
//    		for(int j =0;j<map->height;j++)
//    		{
//    			if(map->grid[i][j])
//    			{
//	    			QPointF pix(i,j);
//	    			map->convertPix(&pix);
//	    			Rep::Point_2 p(pix.x(),pix.y());    			
//	    			vvd->insert(p);
//					counter++;
//    			}    			
//    		}
//			bool b = vvd->is_valid();
//		    if ( b ) 
//		    {
//	      		qDebug("Voronoi diagram is valid. with:%d Points",counter);
//	      		skeletonGenerated = true;
//	    	} 
//	    	else 
//	    	{
//	      		qDebug("Voronoi diagram is NOT valid.");
//	      		skeletonGenerated = false;
//	    	}    		
//    }
//	else
//	{
//		std::cout<<"\n Map not generated YET";
//	}    
////	std::ifstream in("/home/BlackCoder/workspace/CasPlanner/modules/Voronoi/test.pnts");
////    if ( in )
////    {
////	    while (in >> p) 
////	    {
////	    	vvd->insert(p);
////	      	counter++;
////	    }
////		qDebug("\n%d sites have been inserted...", counter);	
////		bool b = vvd->is_valid();
////	    if ( b ) 
////	    {
////      		qDebug("Voronoi diagram is valid.");
////      		skeletonGenerated = true;
////    	} 
////    	else 
////    	{
////      		qDebug("Voronoi diagram is NOT valid.");
////      		skeletonGenerated = false;
////    	}
////    }
////	else
////	{
////		std::cout<<"\n File Not Found";
////	}
//}

void TasksGui::generateSkeleton()
{
	if(!playGround->mapManager->skeletonGenerated)
	{
		playGround->mapManager->generateSkeleton();
		this->sskel = playGround->mapManager->sskel;
		if (! this->sskel)
		{
			skeletonGenerated = false;		
			qDebug("\nNo Skeleton Generated");
			return;
		}
		else
		{
			skeletonGenerated = true;
			qDebug("\nSkeleton Generated, it contains%d Verticies",playGround->mapManager->mapSkeleton.verticies.size());		
			mapGL.setSSkelPtr(playGround->mapManager->mapSkeleton.getSSkelPtr());
			mapGL.paintGL();
			if (voronoiPlanner)
			{
				delete voronoiPlanner;
			}
			voronoiPlanner = new VoronoiPathPlanner(this->sskel);
			voronoiPlanner->buildSpace();
		}
	}
}

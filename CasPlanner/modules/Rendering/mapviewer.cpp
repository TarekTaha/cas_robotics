#include "mapviewer.h"

MapViewer::MapViewer(QWidget *parent,PlayGround *playG,NavControlPanel *navCo)
 : QGLWidget(QGLFormat(QGL::DoubleBuffer | QGL::Rgba | QGL::SampleBuffers), parent),
 step(1),
 playGround(playG),
 navControlPanel(navCo),
 zoomFactor(15.5),
 xOffset(0),
 yOffset(0),
 zOffset(0),
 yaw(0),
 pitch(0),
 fudgeFactor(3),
 showOGs(true),
 showSnaps(true),
 showLabels(true),
 showGrids(true),
 showRobots(true),
 showPointclouds(true),
 showPatchBorders(true),
 hideGoals(false),
 start_initialized(false),
 end_initialized(false),
 mainMapBuilt(false),
 ogMap(playG->mapManager->globalMap)
{
	// Data Logging Timer
    renderTimer = new QTimer(this);
    connect(renderTimer, SIGNAL(timeout()), this, SLOT(updateGL()));
    renderTimer->start(100);
  	clearColor = Qt::black;
    setFocusPolicy(Qt::StrongFocus);
	connect(this, SIGNAL(setStart(Pose)),  navControlPanel, SLOT(setStart(Pose)));
	connect(this, SIGNAL(setEnd(Pose))  ,  navControlPanel, SLOT(setEnd(Pose)));
	connect(playGround,SIGNAL(mapUpdated(Map*)),this,       SLOT(updateMap(Map*)));
	
  	RGB[0][0] = 0.0; RGB[0][1] = 0.7;   RGB[0][2] = 0.7;   // Lightblue
  	RGB[1][0] = 1.0; RGB[1][1] = 0.51;  RGB[1][2] = 0.278; // Sienna1
  	RGB[2][0] = 0.0; RGB[2][1] = 0.7;   RGB[2][2] = 0.0;   // Green
  	RGB[3][0] = 0.7; RGB[3][1] = 0.7;   RGB[3][2] = 0.0;   // Yellow
  	RGB[5][0] = 1.0; RGB[5][1] = 0.0;   RGB[5][2] = 1.0;   // Magenta
  	RGB[6][0] = 0.0; RGB[6][1] = 0.0;   RGB[6][2] = 0.7;   // Blue
  	RGB[7][0] = 1.0; RGB[7][1] = 0.65;  RGB[7][2] = 0.0;   // Orange
  	RGB[8][0] = 1.0; RGB[8][1] = 0.078; RGB[8][2] = 0.576; // DeepPink
  	RGB[9][0] = 0.8; RGB[9][1] = 0.0;   RGB[9][2] = 0.0;   // Red
//	qDebug("OpenGL Initialized"); fflush(stdout);
}

QSize MapViewer::sizeHint()
{
    return QSize(640,480);
}


QSize MapViewer::minimumSizeHint()
{
    return QSize(320,240);
}

void MapViewer::initializeGL()
{
    // Initialization
    //glShadeModel(GL_SMOOTH);       
    glShadeModel(GL_FLAT);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    //glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
   	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
//    renderText(0,0,0,"");
    glFlush();
    mapList = glGenLists(1);
}

void MapViewer::resizeGL(int w, int h)
{
    screenWidth = w;
    screenHeight = h;
	aspectRatio = ((float) w)/((float) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(180, aspectRatio, 1,1000);
	glViewport(0,0,w,h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glOrtho(-aspectRatio, aspectRatio, -1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	updateGL();
	
//    glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
//    glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
//    glGetIntegerv(GL_VIEWPORT,viewport);
}

void MapViewer::setProvider(MapProvider *)
{

}

void MapViewer::updateMap(Map *newMap)
{
	qDebug("Updating Map");
	this->ogMap = newMap;
    mainMapBuilt = false;
    updateGL();
}

void MapViewer::renderPaths()
{
	if(!playGround)
	{
		qDebug("WHAT THEEEE !!!");
		exit(1);
	}
	for(int i=0;i<playGround->robotPlatforms.size();i++)
	{
		if(playGround->robotPlatforms[i]->planningManager->pathPlanner->path)
		{
			Node * path = playGround->robotPlatforms[i]->planningManager->pathPlanner->path;
	    	glColor4f(RGB[3][0],RGB[3][1],RGB[3][2],1);
	    	glLineWidth(2);
		    glBegin(GL_LINE_STRIP);
			while(path && path->next)
			{
		    	glVertex2f(path->pose.p.x(),path->pose.p.y());
				path = path->next;
			}
		    glEnd();
		    glLineWidth(1);
		    wayPoint = playGround->robotPlatforms[i]->navigator->wayPoint;
		    // Draw Way Point
		    if(wayPoint.p.x()==0 && wayPoint.p.y()==0)
		    	continue;
		    glPushMatrix();
			    glTranslatef(wayPoint.p.x(),wayPoint.p.y(),0);
			    glRotated(RTOD(wayPoint.phi),0,0,1);
			    glColor4f(1,0,0,0.8);
			    glShadeModel(GL_FLAT);
			    glBegin(GL_TRIANGLE_FAN);
					glColor4f(1,0,0,1);
				    glVertex3f(0,0.1,0);
				    glVertex3f(0.3,0,0);
				    glVertex3f(0,-0.1,0);
			    glEnd();
			glPopMatrix();
		}
	}
}
void MapViewer::setRobotsLocation()
{
	robotsLocation.clear();
	for(int i=0;i<playGround->robotPlatforms.size();i++)
	{
		if(playGround->robotPlatforms[i]->commManager->connected)
			robotsLocation.push_back(playGround->robotPlatforms[i]->commManager->getLocation());
	}
}
void MapViewer::renderLaser()
{
	if(!playGround)
	{
		qDebug("WTFFFF !!!");
		exit(1);
	}
	for(int i=0;i<playGround->robotPlatforms.size();i++)
	{
		if(!playGround->robotPlatforms[i]->commManager->connected)
			continue;		
	    LaserScan laserScan = playGround->robotPlatforms[i]->commManager->getLaserScan();
	    //Pose loc = playGround->robotPlatforms[i]->robot->robotLocation;
	    Pose loc = robotsLocation[i];
	    glPushMatrix();
	    //qDebug("Laser Pose X:%f Y:%f Phi:%f",laserScan.laserPose.p.x(),laserScan.laserPose.p.y(),laserScan.laserPose.phi);
	    glTranslatef(loc.p.x(),loc.p.y(),0);
	    glRotated(RTOD(loc.phi),0,0,1);

	    //glColor3f(0.623,0.811,0.3);
	    glColor4f(0,1.0f/double(i+1),0.2,1.0f/double(i+1));
//	    glBegin(GL_TRIANGLE_FAN);
//	    	glVertex2f(0,0);
//		    if(laserScan.points.size() > 0)
//	    	{
//	        	for(int m=0; m < laserScan.points.size(); m++)
//		        {
//					laserScan.points[m] = Trans2Global(laserScan.points[m],laserScan.laserPose);
//	    	        glVertex2f(laserScan.points[m].x(), laserScan.points[m].y());
//	        	}
//		    }
//	    	glVertex2f(0,0);
//	    glEnd();
	    glBegin(GL_LINE_LOOP);
		    if(laserScan.points.size() > 0)
	    	{
	        	for(int m=0; m < laserScan.points.size(); m++)
		        {
					laserScan.points[m] = Trans2Global(laserScan.points[m],laserScan.laserPose);
	    	        glVertex2f(laserScan.points[m].x(), laserScan.points[m].y());
	        	}
		    }
	    glEnd();
	    glPopMatrix();
	}
}

void MapViewer::renderSearchTree()
{
	for(int i=0;i<1;i++)
	{
		SearchSpaceNode * temp,*child;
		if(!playGround->robotPlatforms[i]->planningManager->renderTree)
			continue;
		temp =  playGround->robotPlatforms[i]->planningManager->pathPlanner->search_space;
	    glPushMatrix();
	    glColor3f(1,0,0);
		while(temp)
		{
			glColor3f(1,0,0);
			for(int j=0;j<temp->children.size();j++)
			{
				//qDebug("J is:%d",j); fflush(stdout);
				child = temp->children[j];
				if(!child)
				{
					qDebug("Why the hell there is an empty CHILD ???");
					fflush(stdout);
					continue;
				}
			    glBegin(GL_LINE_LOOP);
			    	glVertex2f(temp->location.x(),temp->location.y());
			    	glVertex2f(child->location.x(),child->location.y());
			    glEnd();
			}
		    temp = temp->next;
		}
	    glPopMatrix();
	}
}
void MapViewer::renderExpandedTree()
{
	for(int i=0;i<1;i++)
	{
		vector <Tree> tree;
		QPointF child;
		tree =  playGround->robotPlatforms[i]->planningManager->pathPlanner->tree;
	    glPushMatrix();
	    glColor3f(0,1,0);
		for(unsigned int k=0;k<tree.size();k++)
		{
			for(int j=0;j<tree[k].children.size();j++)
			{
				//qDebug("J is:%d",j); fflush(stdout);
				child = tree[k].children[j];
			    glBegin(GL_LINE_LOOP);
			    	glVertex2f(tree[k].location.x(),tree[k].location.y());
			    	glVertex2f(child.x(),child.y());
			    glEnd();
			}
		}
	    glPopMatrix();
	}
}
void MapViewer::renderRobot()
{
	if(!playGround)
	{
		qDebug("WHAT THEEEE !!!");
		exit(1);
	}
	for(int i=0;i<playGround->robotPlatforms.size();i++)
	{
		if(!playGround->robotPlatforms[i]->commManager->connected)
			continue;
	    Pose loc = robotsLocation[i];
//	    qDebug("Robot Location is X:%f Y:%f Phi:%f",robotsLocation[i].p.x(),robotsLocation[i].p.y(),
//	    	   robotsLocation[i].phi);
	    // Render Robot's trail
	    if(playGround->robotPlatforms[i]->robot->robotName=="Static Obstacle")
	    {
		    // Obstacle
		    glPushMatrix();
		    glTranslated(loc.p.x(),loc.p.y(),0);
		    glRotated(RTOD(loc.phi),0,0,1);
		    glShadeModel(GL_FLAT);
			glColor4f(1,0,0,1);
			glBegin(GL_TRIANGLE_FAN);
			for(int m=0;m<playGround->robotPlatforms[i]->robot->local_edge_points.size();m++)
			{
				glVertex2f(playGround->robotPlatforms[i]->robot->local_edge_points[m].x(),playGround->robotPlatforms[i]->robot->local_edge_points[m].y());
			}
			glEnd();
			glPopMatrix();
			continue;
	    }
		if(playGround->robotPlatforms[i]->navigator->trail.size()>1)
		{
	    	glColor4f(RGB[i][0],RGB[i][1],RGB[i][2],1);
		    glBegin(GL_LINE_STRIP);
			    for(int k =0;k<playGround->robotPlatforms[i]->navigator->trail.size();k++)
			    {
			    	glVertex2f(playGround->robotPlatforms[i]->navigator->trail[k].x(),
			    			   playGround->robotPlatforms[i]->navigator->trail[k].y());
			    }
		    glEnd();
		}
	    glPushMatrix();
	    glTranslatef(loc.p.x(),loc.p.y(),0);
	    glRotated(RTOD(loc.phi),0,0,1);
	    glShadeModel(GL_FLAT);
	    // Robot Boundaries BOX
//		glColor4f(0.63,0.58,0.22,1.0f/double(i+1));
//		glColor4f(0.5,0.5/double(i+1),0.5,1.0f/double(i+1));
		glColor4f(RGB[i][0],RGB[i][1],RGB[i][2],1);
		glBegin(GL_TRIANGLE_FAN);
		for(int m=0;m<playGround->robotPlatforms[i]->robot->local_edge_points.size();m++)
		{
			glVertex2f(playGround->robotPlatforms[i]->robot->local_edge_points[m].x(),playGround->robotPlatforms[i]->robot->local_edge_points[m].y());
		}
		glEnd();
        glColor4f(1,1,1,1);
        QFont font40; font40.setPointSize(10);
	    renderText(1.6,0,0, qPrintable(playGround->robotPlatforms[i]->robot->robotName),font40);
	    glBegin(GL_LINE_LOOP);
			glColor4f(0,0,1,0.5);
		    glVertex3f(1.3, 0.15,0);
		    glVertex3f(1.5,0,0);
		    glVertex3f(1.3,-0.15,0);
	    glEnd();

	    glBegin(GL_LINE_LOOP);
			glColor4f(0,0,1,0.5);
		    glVertex3f(0,0,0);
		    glVertex3f(1.5,0,0);
	    glEnd();

		glColor4f(0,1,0,1);
		for (int k=0;k<playGround->robotPlatforms[i]->robot->check_points.size();k++)
		{
			drawCircle(playGround->robotPlatforms[i]->robot->check_points[k],playGround->robotPlatforms[i]->robot->expansionRadius);
		}

	    glPopMatrix();
	}
}

void MapViewer::update()
{
      this->updateGL();
}

void MapViewer::loadTexture()
{	
//	qDebug("oldW:%d oldH:%d",ogMap->width,ogMap->height);	
   	newWidth =  (int) std::pow(2.0f, (int)ceil(log((float)ogMap->width) / log(2.f)));
   	newHeight = (int) std::pow(2.0f, (int)ceil(log((float)ogMap->height) / log(2.f)));
	ratioW  = ((float) ogMap->width)/newWidth;
	ratioH  = ((float) ogMap->height)/newHeight;
//	qDebug("MW:%d MH:%d RatioW:%f RatioH:%f",newWidth,newHeight,ratioW,ratioH);
   	if (newWidth != ogMap->width || newHeight != ogMap->height)
   		ogMap->scale(newWidth,newHeight);
    unsigned char imgData[ogMap->width*ogMap->height*4];
	long int count=0;
    for(int i=0; i < ogMap->width; i++)
    {
		for(int j=0; j < ogMap->height; j++)
		{
		    if(ogMap->grid[i][j] == true)
			{
		    	count++;
				imgData[(j*ogMap->width+i)*4]   = 0;
				imgData[(j*ogMap->width+i)*4+1] = 0;
				imgData[(j*ogMap->width+i)*4+2] = 0;
				imgData[(j*ogMap->width+i)*4+3] = 255;
			}
		    else
		    {
				imgData[(j*ogMap->width+i)*4]   = 255;
				imgData[(j*ogMap->width+i)*4+1] = 255;
				imgData[(j*ogMap->width+i)*4+2] = 255;
				imgData[(j*ogMap->width+i)*4+3] = 255;
		    }
		}
    }
    /* 
     * Not necessary anymore as i am doing the scaling myself
     * in a much more efficient way (I think :) )
     */
//	unsigned char * scaledData;
//   	if (newWidth != ogMap->width && newHeight != ogMap->height)
//   	{
//      	scaledData = new unsigned char[newWidth * newHeight * 4];
//      	if (gluScaleImage(GL_RGBA, ogMap->width, ogMap->height,
//                        GL_UNSIGNED_BYTE, imgData, newWidth, 
//                        newHeight, GL_UNSIGNED_BYTE, scaledData) != 0)
//      	{
//         	delete[] scaledData;
//         	return;
//      	}
//   	}
//   	else
//   		scaledData = imgData;
    glEnable(GL_TEXTURE_2D);       /* Enable Texture Mapping */    
    glGenTextures(1, &texId);    
    glBindTexture(GL_TEXTURE_2D, texId); 
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ogMap->width, ogMap->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, imgData);    
    
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glDisable(GL_TEXTURE_2D);
    mainMapBuilt = true;   
};

///*!
// *  Renders The main Map loaded from the image file
// */
void MapViewer::renderMap()
{
    glNewList(mapList, GL_COMPILE);		
    glEnable(GL_TEXTURE_2D);       /* Enable Texture Mapping */
    glPushMatrix();
    glBindTexture(GL_TEXTURE_2D, texId);
    // Inverse the Y-axis
    glScalef(1,-1,1);
    glTranslatef(-(newWidth*ogMap->mapRes)/2.0f,-(newHeight*ogMap->mapRes)/2.0f,0);
    //glColor4f(1,1,1,0.8);
	// Define Coordinate System
    glBegin(GL_QUADS);
//	    glTexCoord2f(1,float(newHeight)/float(newWidth));			glVertex2f(newWidth*ogMap->mapRes,newHeight*ogMap->mapRes);
//	    glTexCoord2f(1,0.0);		glVertex2f(newWidth*ogMap->mapRes,0.0);
//	    glTexCoord2f(0.0,0.0);		glVertex2f(0.0,0.0);
//	    glTexCoord2f(0.0,float(newHeight)/float(newWidth));    		glVertex2f(0.0,newHeight*ogMap->mapRes);
//
		ratioH = 1;
		ratioW = float(newHeight)/float(newWidth);  
		glTexCoord2f(0.0,0.0);  glVertex2f(0.0,0.0);
		glTexCoord2f(1.0,0.0);  glVertex2f(newWidth*ogMap->mapRes,0.0);
		glTexCoord2f(1.0,1.0);  glVertex2f(newWidth*ogMap->mapRes,newHeight*ogMap->mapRes);
		glTexCoord2f(0.0,1.0);  glVertex2f(0.0,newHeight*ogMap->mapRes);
    glEnd();

    // Surrounding BOX
	glColor4f(0,0,0,0.5);
	glBegin(GL_LINE_LOOP);
		glVertex2f(0,0);
		glVertex2f(0.0,newHeight*ogMap->mapRes);
		glVertex2f(newWidth*ogMap->mapRes,newHeight*ogMap->mapRes);
		glVertex2f(newWidth*ogMap->mapRes,0.0);
	glEnd();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
    glEndList();	    
//	qDebug("HERE MapRender2"); fflush(stdout);    
}

void MapViewer::displayGrid()
{
	showGrids = true;    
	glPushMatrix();
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
		// X-axis indicator
	    int i = int((ogMap->width*ogMap->mapRes)/2.0 + 2);
	    {
		    glBegin(GL_LINE_LOOP);
				glColor4f(0,0,0,0.5);
			    glVertex3f(i-1,0.5,0);
			    glVertex3f(i,0,0);
			    glVertex3f(i-1,-0.5,0);
		    glEnd();
//		    renderText(i,-1,1, "X");
		 }
		 //Y-axis indicator
	    int j = int((ogMap->height*ogMap->mapRes)/2.0 + 2);
	    {
		    glBegin(GL_LINE_LOOP);
				glColor4f(0,0,0,0.5);
			    glVertex3f(-0.5,j-1,0);
			    glVertex3f(0,j,0);
			    glVertex3f(0.5,j-1,0);
		    glEnd();
//		    renderText(1,j,1, "Y");
		 }
    }
    glPopMatrix();	
}

void MapViewer::drawCircle(float radius)
{
   glBegin(GL_LINES);
	   for (int i=0; i < 360; i++)
	   {
	      float degInRad = DTOR(i);
	      glVertex2f(cos(degInRad)*radius,sin(degInRad)*radius);
	   }
   glEnd();
}

void MapViewer::drawCircle(QPointF center,float radius)
{
   glBegin(GL_LINES);
	   for (int i=0; i < 360; i++)
	   {
	      float degInRad = DTOR(i);
	      glVertex2f(cos(degInRad)*radius+center.x(),sin(degInRad)*radius+center.y());
	   }
   glEnd();
}

void MapViewer::renderObservation()
{
	if(!playGround->mapManager || !playGround->robotPlatforms[0]->intentionRecognizer||!playGround->robotPlatforms[0]->commManager)
		return;
	glPushMatrix();
		glTranslatef(-0.7,0.8,0.0);
		glScalef(1/15.0, 1/15.0, 1.0);
		drawCircle(1.0);
		int obs = playGround->robotPlatforms[0]->intentionRecognizer->observation;
		glLineWidth(2);
		switch (obs)
		{
			case 0:
   				glBegin(GL_LINES);			
					glVertex2f( 0.0, 0.0);
					glVertex2f( 0.0, 1.0);
				glEnd();	
				glBegin(GL_TRIANGLE_FAN);
					glVertex2f(-0.2, 0.8);
					glVertex2f( 0.0, 1.0);
					glVertex2f( 0.2, 0.8);			
					glVertex2f(-0.2, 0.8);
				glEnd();
				break;
			case 1:
				glLineWidth(2);			
   				glBegin(GL_LINES);	
					glVertex2f( 0.0, 0.0);
					glVertex2f( 0.0,-1.0);
   				glEnd();
				glBegin(GL_TRIANGLE_FAN);
					glVertex2f( 0.2,-0.8);
					glVertex2f( 0.0,-1.0);
					glVertex2f(-0.2,-0.8);
					glVertex2f( 0.2,-0.8);			
				glEnd();									
				break;
			case 2:
				glLineWidth(2);			
			   	glBegin(GL_LINES);
					glVertex2f( 0.0, 0.0);
					glVertex2f( 1.0, 0.0);			   	
			   	glEnd();	
				glBegin(GL_TRIANGLE_FAN);
					glVertex2f( 0.8, 0.2);
					glVertex2f( 1.0, 0.0);
					glVertex2f( 0.8,-0.2);			
					glVertex2f( 0.8,-0.2);
				glEnd();																	
				break;
			case 3:
				glLineWidth(2);			
   				glBegin(GL_LINES);
					glVertex2f( 0.0, 0.0);
					glVertex2f(-1.0, 0.0);   					
   				glEnd();
				glBegin(GL_TRIANGLE_FAN);
					glVertex2f(-0.8, 0.2);
					glVertex2f(-1.0,-0.0);
					glVertex2f(-0.8,-0.2);
					glVertex2f(-0.8, 0.2);			
				glEnd();																	
				break;
			default:
				drawCircle(0.2);				
		}
	glLineWidth(1);		
	glPopMatrix();
}

void MapViewer::renderAction()
{
	if(!playGround->mapManager || !playGround->robotPlatforms[0]->intentionRecognizer||!playGround->robotPlatforms[0]->commManager)
		return;	
}

void MapViewer::renderSpatialStates()
{
	if(!playGround->mapManager || !playGround->robotPlatforms[0]->intentionRecognizer||!playGround->robotPlatforms[0]->commManager)
		return;
	// This is not the general Case now and i might need to change it
	Pose l = playGround->robotPlatforms[0]->commManager->getLocation();
	for(int i=0; i < playGround->mapManager->mapSkeleton.verticies.size() ;i++)
	{
	    glPushMatrix();
	    glTranslated(playGround->mapManager->mapSkeleton.verticies[i].location.x(),playGround->mapManager->mapSkeleton.verticies[i].location.y(),0);
	    glShadeModel(GL_FLAT);
	    if(i==playGround->mapManager->mapSkeleton.getCurrentSpatialState(l))
	    	glColor4f(1,0,0,0.5);
	    else if (i == playGround->robotPlatforms[0]->intentionRecognizer->nextState)
	    	glColor4f(0,0,1,1);
	    else
	    	glColor4f(0.33,0.33,0.33,0.5);
		glRectf(-0.2f,0.2f, 0.2f, -0.2f);
		
		if( playGround->mapManager->mapSkeleton.destIndexes.indexOf((i%playGround->mapManager->mapSkeleton.numStates))!=-1 )
		{
			glColor4f(1.0,0.33,0.33,0.5);
			glBegin(GL_LINE);
				glVertex2f(-0.2f, 0.2f);
				glVertex2f( 0.2f,-0.2f);
				glVertex2f(-0.2f,-0.2f);
				glVertex2f( 0.2f, 0.2f);
			glEnd();
		}
		glPopMatrix();
	}

}

void MapViewer::renderDestIndicators()
{
	if(!playGround->mapManager || !playGround->robotPlatforms[0]->intentionRecognizer ||!playGround->robotPlatforms[0]->commManager)
		return;
		
//	This is not the general Case now and i might need to change it
//	Pose l = playGround->robotPlatforms[0]->commManager->getOdomLocation();
	Pose l = playGround->robotPlatforms[0]->commManager->getLocation();
	for(int i=0; i < playGround->robotPlatforms[0]->intentionRecognizer->numDestinations ;i++)
	{
		int r;
		r = playGround->mapManager->mapSkeleton.destIndexes[i];
//		drawProbHisto(playGround->mapManager->mapSkeleton.verticies[r].location, playGround->robotPlatforms[0]->intentionRecognizer->destBelief[i]);
		glPushMatrix();
			glTranslatef(-0.5,0.9 - i*0.05 ,0.0);
			glColor4f(1.0f,0.5f,0.0f,1.0f);
			glScalef(playGround->robotPlatforms[0]->intentionRecognizer->destBelief[i]*4.0f,1.0f, 1.0f);
			glRectf(0.0,0.025f, 0.025f, 0.0f);
		glPopMatrix();
	}	
}

void MapViewer::drawProbHisto(QPointF pos, double prob)
{
	QString str = QString("%1 \%").arg((int)(prob*100));  
	if(prob==0)
		return;
	glPushMatrix();
	glTranslatef(pos.x(),pos.y(),0.0f);
//  glColor4f(1.0f,0.5f,0.0f,1.0f);
// 	renderText(0 ,0 + 0.2, prob+ 0.2, str);
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

void MapViewer::showIndicators()
{
	//if(!hideGoals)
	{
	    if(start_initialized)
	    {
		    glPushMatrix();
		    glTranslatef(start.p.x(),start.p.y(),0);
		    glRotated(RTOD(start.phi),0,0,1);
		    glColor4f(1,1,1,0.8);
		    glShadeModel(GL_FLAT);
		    // Path Start
		    glBegin(GL_TRIANGLE_FAN);
				glColor4f(0,1,0,1);
			    glVertex3f(-0.2,0.15,0);
			    glVertex3f(0.3,0,0);
			    glVertex3f(-0.2,-0.15,0);
		    glEnd();
			glPopMatrix();
		    glPushMatrix();
		    if(step == 2)
		    {
			    glBegin(GL_LINE_LOOP);
					glColor4f(1,1,1,1);
				    glVertex3f(start.p.x(),start.p.y(),0);
				    glVertex3f(mousePos.x(),mousePos.y(),0);
			    glEnd();
		    }
		    glColor4f(0,0,0,0.8);
//	        QFont font40; font40.setPointSize(14);
//	        renderText(0.2,0,0,QString("Start"), font40);
			glPopMatrix();
	    }
	    if(end_initialized)
	    {
		    glPushMatrix();
		    glTranslatef(end.p.x(),end.p.y(),0);
		    glRotated(RTOD(end.phi),0,0,1);
		    glColor4f(1,1,1,0.8);
		    glShadeModel(GL_FLAT);
		    // Path End
		    glBegin(GL_TRIANGLE_FAN);
				glColor4f(0,1,0,1);
			    glVertex3f(-0.2,0.15,0);
			    glVertex3f(0.3,0,0);
			    glVertex3f(-0.2,-0.15,0);
		    glEnd();
			glPopMatrix();
		    glPushMatrix();
		    if(step == 4)
		    {
			    glBegin(GL_LINE_LOOP);
					glColor4f(1,1,1,1);
				    glVertex3f(end.p.x(),end.p.y(),0);
				    glVertex3f(mousePos.x(),mousePos.y(),0);
			    glEnd();
		    }
		    glColor4f(0,0,0,0.8);
//	        QFont font40; font40.setPointSize(14);
//	        renderText(0.2,0,0,QString("End"), font40);
			glPopMatrix();
	    }
	}	
}

void MapViewer::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    glBlendFunc(GL_DST_COLOR, GL_ZERO);
    glEnable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
//    glDisable(GL_DEPTH_TEST);
//    glEnable(GL_POINT_SMOOTH);
//    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
//    glEnable(GL_LINE_SMOOTH);
//    glEnable(GL_POLYGON_SMOOTH);
	
	renderObservation();
	renderDestIndicators();	
   	glPushMatrix();
    glScalef(1/zoomFactor, 1/zoomFactor, 1/zoomFactor);
    glColor4f(1,1,1,1);
//    renderText(zoomFactor*aspectRatio*0.90-1, -0.9*zoomFactor, 0, "grid: 1 m");

    glRotatef(pitch,1,0,0);
    glRotatef(yaw,0,0,1);
    glTranslatef(xOffset, yOffset, zOffset);

    glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
    glGetIntegerv(GL_VIEWPORT,viewport);

	if(this->ogMap)
	{
		displayGrid();
		showIndicators();
		renderPaths();
	    renderRobot();	    
		setRobotsLocation();	
//	    renderLaser();      
	    renderSpatialStates();
	    renderObservation();
	    renderAction();
//	  	renderSearchTree();
//		renderExpandedTree();	
		if(!mainMapBuilt)
		{
			loadTexture();
			renderMap();
		}
	}		
    glCallList(mapList);
        
    //glDisable(GL_BLEND);
    //glEnable(GL_DEPTH_TEST);
    //glDisable(GL_POINT_SMOOTH);
    //glDisable(GL_LINE_SMOOTH);
    //glDisable(GL_POLYGON_SMOOTH);
    glPopMatrix();
}

void MapViewer::setShowOGs(int state)
{
    if(state==0)
    {
		showOGs = false;
    }
    else
    {
		showOGs = true;
    }
    update();
}

void MapViewer::setShowSnaps(int state)
{
    if(state==0)
    {
		showSnaps = false;
    }
    else
    {
		showSnaps = true;
    }
    update();
}
void MapViewer::setShowGrids(int state)
{
    if(state==0)
    {
		showGrids = false;
    }
    else
    {
		showGrids = true;
    }
    update();
}

void MapViewer::setShowRobots(int state)
{
    if(state==0)
    {
		showRobots = false;
    }
    else
    {
		showRobots = true;
    }
    update();
}

void MapViewer::setShowPointclouds(int state)
{
    if(state==0)
    {
		showPointclouds = false;
    }
    else
    {
		showPointclouds = true;
    }
    update();
}

void MapViewer::setShowPatchBorders(int state)
{
    if(state==0)
    {
		showPatchBorders = false;
    }
    else
    {
		showPatchBorders = true;
    }
    update();
}
void MapViewer::mouseDoubleClickEvent(QMouseEvent *me)
{
//	updateMap(playGround->robotPlatforms[0]->planningManager->pathPlanner->map);	
	QPointF p(me->x(),me->y());
	//qDebug("Mouse Double click x: %f y: %f",p.x(),p.y());
    p = getOGLPos(p.x(),p.y());
	if(step == 1)
	{
		start.p = p;
		step++;
		start_initialized = true; end_initialized = false;
		setMouseTracking(true);
		hideGoals = false;
	}
	else if(step==3)
	{
		end.p = p;
		end_initialized = true;
		step++;
		setMouseTracking(true);
	}

}

void MapViewer::mousePressEvent(QMouseEvent *me)
{
	QPointF p(me->x(),me->y());
    p = getOGLPos(me->x(),me->y());
    //qDebug("Mouse pressed x: %f y: %f",x,y);
	if(step ==2)
	{
		start.phi = atan2(p.y() - start.p.y(),p.x() - start.p.x());
		emit setStart(start);
		qDebug("Start Angle =%f",RTOD(start.phi));
		step++;
		update();
		setMouseTracking(false);
	}
	else if(step == 4)
	{
		end.phi = atan2(p.y() - end.p.y(),p.x() - end.p.x());
		qDebug("End Angle =%f",RTOD(end.phi));
		emit setEnd(end)	;
		end_initialized = true;
		step = 1;
		update();
		setMouseTracking(false);
	    hideGoals = true;
	}
}

void MapViewer::mouseMoveEvent ( QMouseEvent * me )
{
	QPointF p(me->x(),me->y());
    mousePos = getOGLPos(me->x(),me->y());
	update();
}

QPointF MapViewer::getOGLPos(double x, double y)
{
	QPointF retval;
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;

	winX = x;
	winY = (float)viewport[3] - y;
	glReadPixels( (int)x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
	gluUnProject( winX, winY, winZ, modelMatrix, projMatrix, viewport, &posX, &posY, &posZ);
//	qDebug("Translated to x: %f y: %f z:%f",posX,posY,posZ);
    position[0] = posX;
    position[1] = posY;
  	retval.setX(position[0]);
  	retval.setY(position[1]);
  	return retval;
}

void MapViewer::mouseReleaseEvent(QMouseEvent *)
{
}

void MapViewer::keyPressEvent(QKeyEvent *e)
{
    if(e->key() == Qt::Key_C)
    {
		if(e->modifiers() && Qt::ShiftModifier)
		{
			for(int i=0;i<playGround->robotPlatforms.size();i++)
			{
				playGround->robotPlatforms[i]->navigator->trail.clear();
			}
		}
		else
		{

		}
    }
    else if(e->key() == Qt::Key_W)
    {
		if(e->modifiers() && Qt::ShiftModifier)
		{
		    emit moveMOUp();
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
		    emit moveMODown();
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
		    emit moveMOLeft();
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
		    emit moveMORight();
		}
		else
		{
		    xOffset += 0.1*zoomFactor;
		}
    }
    else if(e->key() == Qt::Key_BracketLeft)
    {
		zoomFactor *= 1.1;
    }
    else if(e->key() == Qt::Key_BracketRight)
    {
		zoomFactor /= 1.1;
    }
    else if(e->key() == Qt::Key_Left)
    {
		if(e->modifiers() && Qt::ShiftModifier)
		{
		    emit yawMOPos();
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
		    emit yawMONeg();
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

void MapViewer::focusInEvent(QFocusEvent *)
{
    makeCurrent();
   	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    updateGL();
}

void MapViewer::focusOutEvent(QFocusEvent *)
{
    makeCurrent();
   	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    updateGL();
}

QImage MapViewer::captureMap()
{
    return grabFrameBuffer();
}
void MapViewer::saveImage()
{
    bool ok;
    QString filename = QInputDialog::getText(this, "Image Capture","Enter a name for the Image:", QLineEdit::Normal,
	    QString::null, &ok);
	const char * type = "PNG";
    sleep(1);
    if(ok && !filename.isEmpty())
    {
		QImage capturedMap = this->captureMap();
		capturedMap.save(filename,type,-1);
    }
}
MapViewer::~MapViewer()
{

}

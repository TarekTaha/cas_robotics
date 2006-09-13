#include "mapviewer.h"

MapViewer::MapViewer(QWidget *parent,PlayGround *playG,NavControlPanel *navCo)
 : QGLWidget(QGLFormat(QGL::AlphaChannel), parent),
 step(1),
 playGround(playG),
 navControlPanel(navCo),
 zoomFactor(10),
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
 start_initialized(false),
 end_initialized(false),
 mainMapBuilt(false),
 mapName(playG->mapName)
{
	// Data Logging Timer
    renderTimer = new QTimer(this);
    connect(renderTimer, SIGNAL(timeout()), this, SLOT(updateGL()));
    renderTimer->start(100);
//	qDebug("Initializing OpenGL"); fflush(stdout);
  	clearColor = Qt::black;
    setFocusPolicy(Qt::StrongFocus);
    glGenTextures(1, &texId); 
	connect(this, SIGNAL(setStart(Pose)),  navControlPanel, SLOT(setStart(Pose)));
	connect(this, SIGNAL(setEnd(Pose))  ,  navControlPanel, SLOT(setEnd(Pose)));
	connect(this, SIGNAL(setMap(QImage)),  navControlPanel, SLOT(setMap(QImage)));	    
	if(!loadImage(mapName))
	{
		qDebug("Error Loading Image");
		exit(1);
	}
	emit setMap(image);
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

int MapViewer::loadImage(QString name)
{
	if(!image.load(name, 0))
	{
		return -1;
	}
	mapData = mapManager.provideMapOG(image,0.05,Pose(0,0,0),false);	
	return 1;
}

QSize MapViewer::sizeHint()
{
    return QSize(640,480);   
}

QSize MapViewer::minimumSizeHint()
{
    return QSize(320,240);   
}

void MapViewer::setMapName(QString name)
{
	this->mapName = name;
}

void MapViewer::initializeGL()
{
	glEnable(GL_TEXTURE_2D);				// Enable Texture Mapping
	glShadeModel(GL_SMOOTH);				// Enable Smooth Shading
	glClearColor(0.70f, 0.7f, 0.7f, 1.0f);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClearDepth(1.0f);						// Depth Buffer Setup
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);		// Set The Blending Function For Translucency
	glEnable(GL_BLEND);						// Enable Blending    
    renderText(0,0,0,""); 
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
    gluPerspective(60, aspectRatio, 1,1000); 
    glMatrixMode(GL_MODELVIEW); 
    glLoadIdentity();
    glTranslatef(0,0,-2);
    glViewport(0,0,w,h); 
    updateGL();
    glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
    glGetIntegerv(GL_VIEWPORT,viewport);    
}
void  MapViewer::SetMapFileName(QString name)
{
	this->mapName = name;	
}

QImage MapViewer::getImage()
{
	return this->image;
}

void MapViewer::setProvider(MapProvider *)
{
    
}

void MapViewer::updateMap(Map *newMap)
{
	qDebug("Updating Map");
	if(this->mapData)
		delete mapData;
	this->mapData = newMap;
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
		if(i>0) continue;
		if(playGround->robotPlatforms[i]->planningManager->pathPlanner->path)
		{
			Node * path = playGround->robotPlatforms[i]->planningManager->pathPlanner->path;
	    	glColor4f(1,1,1,1);
	    	//glColor4f(RGB[3][0],RGB[3][1],RGB[3][2],1);
		    glBegin(GL_LINE_STRIP);
			while(path && path->next)
			{
		    	glVertex2f(path->pose.p.x(),path->pose.p.y());
				path = path->next;
			}
		    glEnd();
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
				    glVertex3f(-0.2,0.15,0); 			
				    glVertex3f(0.1,0,0); 			    	
				    glVertex3f(-0.2,-0.15,0); 			    				    
			    glEnd();
			glPopMatrix();  
		}
	}	
}

void MapViewer::renderLaser()
{
	if(!playGround)
	{
		qDebug("WHAT THEEEE !!!");
		exit(1);
	}
	for(int i=0;i<playGround->robotPlatforms.size();i++)
	{
		// Just draw laser for the first robot, just for the ICRA paper display
		if (i>0) continue;
	    LaserScan laserScan = playGround->robotPlatforms[i]->commManager->getLaserScan(); 
	    //Pose loc = playGround->robotPlatforms[i]->robot->robotLocation;	
	    Pose loc = playGround->robotPlatforms[i]->commManager->getOdomLocation();;	
	    glPushMatrix(); 
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

void MapViewer::renderRobot()
{
	if(!playGround)
	{
		qDebug("WHAT THEEEE !!!");
		exit(1);
	}
	for(int i=0;i<playGround->robotPlatforms.size();i++)
	{
//	    Pose loc = playGround->robotPlatforms[i]->robot->robotLocation;
	    Pose loc = playGround->robotPlatforms[i]->commManager->getOdomLocation();
	    // Render Robot's trail
	    if(playGround->robotPlatforms[i]->robot->robotName=="Static Obstacle")
	    {
		    // Obstacle
		    glPushMatrix();
		    glTranslatef(loc.p.x(),loc.p.y(),0);
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
	    renderText(1.6,0,0, qPrintable(playGround->robotPlatforms[i]->robot->robotName));		
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
	    
	    glPopMatrix();
	}
}

void MapViewer::update()
{
      this->updateGL();
}
void MapViewer::renderMapPatch(Map * mapPatch)
{
    int mapWidth,mapHeight; 
    float ratioW, ratioH; 	
//	qDebug("Rendering Map Patch");
	#ifdef NOPOTD
	   mapWidth  = powl(2,  ceill(log(mapPatch->width)/log(2)));
	   mapHeight = powl(2, ceill(log(mapPatch->height)/log(2)));
	   ratioW  = ((float) mapPatch->width))/mapWidth; 
	   ratioH  = ((float) mapPatch->height))/mapHeight; 
	#else 
	    mapWidth  = mapPatch->width;
	    mapHeight = mapPatch->height;
	    ratioW = 1; 
	    ratioH = 1; 
	#endif	
    unsigned char imgData[mapWidth*mapHeight*4];
	long int count=0;
    for(int i=0; i < mapWidth; i++)
    {
		for(int j=0; j < mapHeight; j++)
		{
		    if(mapPatch->data[i][j] == true)
			{
		    	count++;
				imgData[(j*mapPatch->width+i)*4]   = 255;
				imgData[(j*mapPatch->width+i)*4+1] = 0; 
				imgData[(j*mapPatch->width+i)*4+2] = 0; 
				imgData[(j*mapPatch->width+i)*4+3] = 255;
			}
		    else 
		    {
				imgData[(j*mapPatch->width+i)*4] = 0;  
				imgData[(j*mapPatch->width+i)*4+1] = 0; 
				imgData[(j*mapPatch->width+i)*4+2] = 0;  
				imgData[(j*mapPatch->width+i)*4+3] = 0;   
		    }
		}
    }
//    qDebug(" Count %ld",count);
    
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texId);  //Select our texture
   
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, mapPatch->width, mapPatch->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, imgData);
  

    glPushMatrix();
    
    // Inverse the Y-axis    
    glScalef(1,-1,1);
    glTranslatef(-(mapPatch->width*mapPatch->resolution)/2.0,-(mapPatch->height*mapPatch->resolution)/2.0,0);
//    glColor4f(0,0,1,0.8);
    glColor4f(1,1,1,0.8);
    glShadeModel(GL_FLAT);
	// Define Coordinate System
    glBegin(GL_QUADS);
	    glTexCoord2f(ratioW,ratioH);	glVertex2f(mapPatch->width*mapPatch->resolution,mapPatch->height*mapPatch->resolution);
	    glTexCoord2f(ratioW,0.0);		glVertex2f(mapPatch->width*mapPatch->resolution,0.0);
	    glTexCoord2f(0.0,0.0);			glVertex2f(0.0,0.0);
	    glTexCoord2f(0.0,ratioH);    	glVertex2f(0.0,mapPatch->height*mapPatch->resolution);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    
    // Surrounding BOX
	glColor4f(1,1,1,0.5); 
	glBegin(GL_LINE_LOOP); 
		glVertex2f(0,0);
		glVertex2f(0.0,mapPatch->height*mapPatch->resolution);
		glVertex2f(mapPatch->width*mapPatch->resolution,mapPatch->height*mapPatch->resolution);
		glVertex2f(mapPatch->width*mapPatch->resolution,0.0);
	glEnd();

    glPopMatrix();	
}
/*!
 *  Renders The main Map loaded from the image file
 */
void MapViewer::renderMap()
{
    int mapWidth,mapHeight; 
    float ratioW, ratioH; 	
//	qDebug("Rendering Map");
	#ifdef NOPOTD
	   mapWidth  = powl(2,  ceill(log(mapData->width)/log(2)));
	   mapHeight = powl(2, ceill(log(mapData->height)/log(2)));
	   ratioW  = ((float) mapData->width))/mapWidth; 
	   ratioH  = ((float) mapData->height))/mapHeight; 
	#else 
	    mapWidth  = mapData->width;
	    mapHeight = mapData->height;
	    ratioW = 1; 
	    ratioH = 1; 
	#endif	
    unsigned char imgData[mapWidth*mapHeight*4];
	long int count=0;
    for(int i=0; i < mapWidth; i++)
    {
		for(int j=0; j < mapHeight; j++)
		{
		    if(mapData->data[i][j] == true)
			{
		    	count++;
//				imgData[(j*mapData->width+i)*4] = 0;
//				imgData[(j*mapData->width+i)*4+1] =0; 
//				imgData[(j*mapData->width+i)*4+2] = 255; 
//				imgData[(j*mapData->width+i)*4+3] = 255;
				imgData[(j*mapData->width+i)*4]   = 255;
				imgData[(j*mapData->width+i)*4+1] = 255; 
				imgData[(j*mapData->width+i)*4+2] = 255; 
				imgData[(j*mapData->width+i)*4+3] = 255;
			}
		    else 
		    {
				imgData[(j*mapData->width+i)*4] = 0;  
				imgData[(j*mapData->width+i)*4+1] = 0; 
				imgData[(j*mapData->width+i)*4+2] = 0;  
				imgData[(j*mapData->width+i)*4+3] = 0;   
		    }
		}
    }
    glNewList(mapList, GL_COMPILE);	

    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, texId);  //Select our texture
   
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
   
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, mapData->width, mapData->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, imgData);
  

    glPushMatrix();
    
    // Inverse the Y-axis    
    glScalef(1,-1,1);
    glTranslatef(-(mapData->width*mapData->resolution)/2.0,-(mapData->height*mapData->resolution)/2.0,0);
//    glColor4f(0,0,1,0.8);
    glColor4f(1,1,1,0.8);
    glShadeModel(GL_FLAT);
	// Define Coordinate System
    glBegin(GL_QUADS);
	    glTexCoord2f(ratioW,ratioH);	glVertex2f(mapData->width*mapData->resolution,mapData->height*mapData->resolution);
	    glTexCoord2f(ratioW,0.0);		glVertex2f(mapData->width*mapData->resolution,0.0);
	    glTexCoord2f(0.0,0.0);			glVertex2f(0.0,0.0);
	    glTexCoord2f(0.0,ratioH);    	glVertex2f(0.0,mapData->height*mapData->resolution);
    glEnd();

    glDisable(GL_TEXTURE_2D);
    
    // Surrounding BOX
	glColor4f(1,1,1,0.5); 
	glBegin(GL_LINE_LOOP); 
		glVertex2f(0,0);
		glVertex2f(0.0,mapData->height*mapData->resolution);
		glVertex2f(mapData->width*mapData->resolution,mapData->height*mapData->resolution);
		glVertex2f(mapData->width*mapData->resolution,0.0);
	glEnd();

    glPopMatrix();
    glEndList();
    mainMapBuilt = true;
}
void MapViewer::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH); 
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST); 
    glEnable(GL_LINE_SMOOTH); 
    glEnable(GL_POLYGON_SMOOTH);
//    glMatrixMode(GL_MODELVIEW);
//    glFlush();
//    
//    glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
//    glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
//    glGetIntegerv(GL_VIEWPORT,viewport); 
    
    glPushMatrix();
    glScalef(1/zoomFactor, 1/zoomFactor, 1/zoomFactor);
    glColor4f(0,0,0,1); 
   
    renderText(zoomFactor*aspectRatio*0.90-1, -0.9*zoomFactor, 0, "grid: 1 m");
    
    glRotatef(pitch,1,0,0); 
    glRotatef(yaw,0,0,1); 
 
    glTranslatef(xOffset, yOffset, zOffset);

//	getOGLPos(mouseDouble.x(),mouseDouble.x());

    if(showGrids)
    {
		for(int i=-(int) zoomFactor*3; i < (int) zoomFactor*3; i++)
		{
		    glBegin(GL_LINES);
			    if(i==0)
			    {
//					glColor4f(0,0,0,0.5);  
					glColor4f(1,1,1,0.5);  
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
	    int i = int((mapData->width*mapData->resolution)/2.0 + 2);
	    {
		    glBegin(GL_LINE_LOOP);
//				glColor4f(0,0,0,0.5);  
				glColor4f(1,1,1,0.5);  
			    glVertex3f(i-1,0.5,0); 			    	
			    glVertex3f(i,0,0); 			    	
			    glVertex3f(i-1,-0.5,0); 				    
		    glEnd();
		    renderText(i,-1,0, "X");		    	    
		 }
		 //Y-axis indicator
	    int j = int((mapData->height*mapData->resolution)/2.0 + 2);
	    {
		    glBegin(GL_LINE_LOOP);
//				glColor4f(0,0,0,0.5);  
				glColor4f(1,1,1,0.5);  
			    glVertex3f(-0.5,j-1,0); 			    	
			    glVertex3f(0,j,0); 				    				    			    
			    glVertex3f(0.5,j-1,0); 				    
		    glEnd();
		    renderText(1,j,0, "Y");
		 }		 
    }

    glColor4f(0,0,0,1.0);
	if(!mainMapBuilt)
	    renderMap();
    glCallList(mapList); 	    
    renderLaser();
    renderRobot();
    renderPaths();
    //renderSearchTree();
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
		glPopMatrix();   	
    }    
		
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_POINT_SMOOTH); 
    glDisable(GL_LINE_SMOOTH); 
    glDisable(GL_POLYGON_SMOOTH);
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
	QPointF p(me->x(),me->y());
//	qDebug("Mouse Double click x: %f y: %f",p.x(),p.y()); 
    p = getOGLPos(p.x(),p.y());		
	if(step == 1)
	{
		start.p = p;
		step++;
		start_initialized = true; end_initialized = false;
		setMouseTracking(true);		
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
//		qDebug("Start Angle =%f",RTOD(start.phi));				
		step++;
		update();
		setMouseTracking(false);
	}
	else if(step == 4)
	{
		end.phi = atan2(p.y() - end.p.y(),p.x() - end.p.x());
//		qDebug("End Angle =%f",RTOD(end.phi));	
		emit setEnd(end)	;
		end_initialized = true;
		step = 1;
		update();
		setMouseTracking(false);		
	}
}

void MapViewer::mouseMoveEvent ( QMouseEvent * me )
{
	QPointF p(me->x(),me->y());	
    mousePos = getOGLPos(me->x(),me->y());	
//    mousePos = getOGLPos(p);
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
//    qDebug("Translated to x: %f y: %f z:%f",posX*0.02,posY*0.02,posZ*0.02); 	
    position[0] = posX*0.02;
    position[1] = posY*0.02;
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
		    trail.clear();
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
//    glClearColor(0.7f,0.7f,0.7f,1.0f);   
    glClearColor(0.0f,0.0f,0.0f,1.0f);   
    updateGL();
}

void MapViewer::focusOutEvent(QFocusEvent *)
{
    makeCurrent();  
//    glClearColor(0.7f,0.7f,0.7f,1.0f);
    glClearColor(0.0f,0.0f,0.0f,1.0f);       
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

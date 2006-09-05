#include "mapviewer.h"

MapViewer::MapViewer(QWidget *parent,RobotManager *rob,QString map_name)
 : QGLWidget(QGLFormat(QGL::AlphaChannel), parent),
 step(1),
 robotManager(rob),
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
 mapName(map_name)
{
  	clearColor = Qt::black;
    setFocusPolicy(Qt::StrongFocus);
    glGenTextures(1, &texId); 
	if(!loadImage(mapName))
	{
		qDebug("Error Loading Image");
		exit(1);
	}
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
//	glShadeModel(GL_SMOOTH);				// Enable Smooth Shading
//	glClearColor(0.70f, 0.7f, 0.7f, 1.0f);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClearDepth(1.0f);						// Depth Buffer Setup
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations
	glBlendFunc(GL_SRC_ALPHA,GL_ONE);		// Set The Blending Function For Translucency
	glEnable(GL_BLEND);						// Enable Blending    
    renderText(0,0,0,""); 
    glFlush();
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

void MapViewer::setWayPoint(Pose *wayPoint)
{
	this->wayPoint = *wayPoint;
}

Pose MapViewer::getStart()
{
	return this->start;	
}

Pose MapViewer::getEnd()
{
	return this->end;
}

QImage MapViewer::getImage()
{
	return this->image;
}

void MapViewer::setProvider(MapProvider *)
{
    
}
void MapViewer::renderLaser()
{
//	double maxRange = 8.0;
    LaserScan laserScan = robotManager->commManager->getLaserScan(); 
    Pose loc = robotManager->robot->robotLocation;	
    glPushMatrix(); 
    glTranslatef(loc.p.x(),loc.p.y(),0);
    glRotated(RTOD(loc.phi),0,0,1);    
//    glBegin(GL_QUADS); 
//    	glVertex2f(-maxRange, -maxRange); 
//	    glVertex2f(-maxRange, maxRange); 
//	    glVertex2f(maxRange, maxRange); 
//    	glVertex2f(maxRange, -maxRange); 
////    	glVertex2f(0, 0); 
////	    glVertex2f(0, 1); 
////	    glVertex2f(1, 1); 
////    	glVertex2f(1, 0); 
//    glEnd(); 
    
    glColor3f(0.623,0.811,0.3); 
    
    glBegin(GL_TRIANGLE_FAN);
    	glVertex2f(0,0);  
	    if(laserScan.points.size() > 0)
    	{
        	for(int i=0; i < laserScan.points.size(); i++)
	        {
				laserScan.points[i] = Trans2Global(laserScan.points[i],laserScan.laserPose);	        	
    	        glVertex2f(laserScan.points[i].x(), laserScan.points[i].y());  
        	}
	    }
    	glVertex2f(0,0);
    glEnd(); 
    glPopMatrix();	
}
void MapViewer::renderRobot()
{
    Pose loc = robotManager->robot->robotLocation;	
	if(trail.size()==0 && loc.p!=QPointF(0,0))
	{
		trail.push_back(loc.p);		
	}
	else if(trail[trail.size()-1]!=loc.p)
	{
		trail.push_back(loc.p);
	}
	if(trail.size()>1)
	{
    	glColor4f(1,1,1,1);
	    glBegin(GL_LINE_STRIP);
		    for(int i =0;i<trail.size();i++)
		    {
		    	glVertex2f(trail[i].x(),trail[i].y());
		    }
	    glEnd();
	}
    glPushMatrix();
    glTranslatef(loc.p.x(),loc.p.y(),0);
    glRotated(RTOD(loc.phi),0,0,1);
    glColor4f(0,0,1,0.8);
    glShadeModel(GL_FLAT);
    // Robot Boundaries BOX
	glColor4f(1,1,1,0.5); 
	glBegin(GL_TRIANGLE_FAN);
	for(int i=0;i<robotManager->robot->local_edge_points.size();i++)
	{
		glVertex2f(robotManager->robot->local_edge_points[i].x(),robotManager->robot->local_edge_points[i].y());
	}
	glEnd();

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
    
    renderText(1.6,0,0, "Robot Direction");		    	    

    glPopMatrix();
}

void MapViewer::update()
{
      this->updateGL();
}
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

    renderMap();
    renderLaser();
    renderRobot();
    if(start_initialized)
    {
	    glPushMatrix();
	    glTranslatef(start.p.x(),start.p.y(),0);
	    glRotated(RTOD(start.phi),0,0,1);
	    glColor4f(1,1,1,0.8);
	    glShadeModel(GL_FLAT);
	    // Path Start
	    glBegin(GL_TRIANGLE_FAN);
			glColor4f(1,1,1,1);  
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
			glColor4f(1,1,1,1);  
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
    // Draw Way Point
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
//		qDebug("Start Angle =%f",RTOD(start.phi));				
		step++;
		update();
		setMouseTracking(false);
	}
	else if(step == 4)
	{
		end.phi = atan2(p.y() - end.p.y(),p.x() - end.p.x());
//		qDebug("End Angle =%f",RTOD(end.phi));		
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

MapViewer::~MapViewer()
{

}

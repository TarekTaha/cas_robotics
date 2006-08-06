#include "mapviewer.h"

MapViewer::MapViewer(QWidget *parent)
 : QGLWidget(QGLFormat(QGL::AlphaChannel), parent),
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
 showPatchBorders(true)
{
  	clearColor = Qt::black;
    setFocusPolicy(Qt::StrongFocus);
    makeCurrent(); 
    glGenTextures(1, &texId); 
    qWarning("Initialized !!!"); fflush(stdout);	
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
	glClearColor(0.70f, 0.7f, 0.7f, 1.0f);
    renderText(0,0,0,""); 
//    glEnable(GL_DEPTH_TEST);
//    glEnable(GL_CULL_FACE);
//    glEnable(GL_TEXTURE_2D);
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
    //qDebug("Aspect ratio set to %f", aspectRatio); 
    //glOrtho(-aspectRatio, aspectRatio, -1, 1, -1000, 1000); 
    gluPerspective(60, aspectRatio, 1,1000); 
    glMatrixMode(GL_MODELVIEW); 
    glLoadIdentity();
    glTranslatef(0,0,-2);
    glViewport(0,0,w,h); 
    updateGL();
}

void MapViewer::setProvider( MapProvider *prov)
{
    provider = prov;
}

void MapViewer::update()
{
      mapData = provider->provideMap();
      this->updateGL(); 
}
void MapViewer::renderMap()
{
	qDebug("Rendering Map");
    unsigned char imgData[mapData.width*mapData.height*4];
    int min=255,max=0; 
    float mean=0; 
//    makeCurrent(); 
//    glPushMatrix(); 
//    glTranslatef(0,0,0); 
//    glScalef(0.5/mapData.width, 0.5/mapData.width, 1.0); 
//    //glScalef(1,1, 1.0); 
//    glColor4f(1.0,1.0,1, 1);
//    glBegin(GL_QUADS); 
//    glVertex2f(-mapData.width, -mapData.width); 
//    glVertex2f(-mapData.width, mapData.width); 
//    glVertex2f(mapData.width, mapData.width); 
//    glVertex2f(mapData.width, -mapData.width); 
    
    for(int i=0; i < mapData.height; i++)
    {
		for(int j=0; j < mapData.width; j++)
		{
		    // qDebug("Pixel %d has value %d", k, (unsigned char) mapData.data[k]);
		    unsigned char val = (unsigned char) mapData.rawData[i*mapData.width+j];
		    if(val < min) min=val;
		    if(val > max) max=val;
		    mean += val;
		    //qDebug("\n VAL:%u",val);
		    if(val > 90)
		    {
		    	//qDebug("Pixel Occupied val=%u",val);
				imgData[(i*mapData.width+j)*4] = 0;
				imgData[(i*mapData.width+j)*4+1] =0; 
				imgData[(i*mapData.width+j)*4+2] = 255; 
				imgData[(i*mapData.width+j)*4+3] = 255;
		    }
		    else if(val < 70)
		    {
//				if(!highlighted)
//				{
//				    imgData[(i*mapData.width+j)*4] = 255;
//				    imgData[(i*mapData.width+j)*4+1] =255; 
//				    imgData[(i*mapData.width+j)*4+2] = 255; 
//				    imgData[(i*mapData.width+j)*4+3] = 50;
//				}
//				else 
				{
				    imgData[(i*mapData.width+j)*4] = 255;
				    imgData[(i*mapData.width+j)*4+1] =255; 
				    imgData[(i*mapData.width+j)*4+2] = 0; 
				    imgData[(i*mapData.width+j)*4+3] = 100;
				}
		    }
		    else 
		    {
				imgData[(i*mapData.width+j)*4] = 0;  
				imgData[(i*mapData.width+j)*4+1] = 0; 
				imgData[(i*mapData.width+j)*4+2] = 0;  
				imgData[(i*mapData.width+j)*4+3] = 0;   
		    }
		}
    }
    
    qDebug("Map values %d %d %f" , min, max, mean/(mapData.width*mapData.height)); 

    glBindTexture(GL_TEXTURE_2D, texId); 
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, mapData.width, mapData.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, imgData);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glEnable(GL_TEXTURE_2D);
    glPushMatrix();
    glScalef(1,-1,1);
    glTranslatef(-(mapData.width*mapData.height)/2.0,-(mapData.width*mapData.height)/2,0);
    glColor4f(0,0,1,0.8);
    glShadeModel(GL_FLAT);
    glBegin(GL_QUADS);
    
    glTexCoord2f(0.0,0.0);
    glVertex2f(0.0,0.0);
    glTexCoord2f(0.0,1);
    glVertex2f(0.0, (mapData.width*mapData.height));
    glTexCoord2f(1, 1);
    glVertex2f(mapData.width*mapData.height,mapData.width*mapData.height);
    glTexCoord2f(1,0.0);
    glVertex2f((mapData.width*mapData.height),0.0);
    glEnd();
    glDisable(GL_TEXTURE_2D);
    
    // We now draw a surrounding rectangle so people know where the edge of the OG map is. 
//    if(showPatchBorders){
//	glColor4f(0,0,1,0.5); 
//	glBegin(GL_LINE_LOOP); 
//	glVertex2f(0,0);
//	glVertex2f(0.0, currentMap->numCellsY*currentMap->metresPerCellY);
//	glVertex2f(currentMap->numCellsX*currentMap->metresPerCellX,currentMap->numCellsY*currentMap->metresPerCellY);
//	glVertex2f(currentMap->numCellsX*currentMap->metresPerCellX,0.0);
//	glEnd();
//    } 
    glPopMatrix();
}
void MapViewer::paintGL()
{
//    qDebug("MAPVIEWER paintGL on mapview called");
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH); 
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST); 
    glEnable(GL_LINE_SMOOTH); 
    glEnable(GL_POLYGON_SMOOTH);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glScalef(1/zoomFactor, 1/zoomFactor, 1/zoomFactor);
    glColor4f(0,0,0,1); 
    glBegin(GL_LINES); 
    glVertex2f(zoomFactor*aspectRatio*0.90-1, -0.9*zoomFactor);
    glVertex2f(zoomFactor*aspectRatio*0.90, -0.9*zoomFactor); 
    glEnd(); 
    renderText(zoomFactor*aspectRatio*0.90-1, -0.9*zoomFactor, 0, "scale: 1 m");
    glRotatef(pitch,1,0,0); 
    glRotatef(yaw,0,0,1); 
 
    glTranslatef(xOffset, yOffset, zOffset);

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
    }
    //Ok, now let's draw dem maps.
    glColor4f(0,0,0,1.0);

    renderMap();
    
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
    if(state==0){
	showSnaps = false;  
    }
    else {
	showSnaps = true; 
    }
    update();
}
void MapViewer::setShowGrids(int state)
{
    if(state==0){
	showGrids = false;  
    }
    else {
	showGrids = true; 
    }
    update();
}

void MapViewer::setShowRobots(int state)
{
    if(state==0){
	showRobots = false;  
    }
    else {
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

void MapViewer::mousePressEvent(QMouseEvent *me)
{
	double x = me->x();
	double y = me->y();
    qDebug("Mouse pressed x: %f y: %f",x,y); 
}

void MapViewer::mouseReleaseEvent(QMouseEvent *me)
{
}
void MapViewer::keyPressEvent(QKeyEvent *e)
{
    qDebug("Key Pressed"); 
    if(e->key() == Qt::Key_W)
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

void MapViewer::focusInEvent(QFocusEvent *fe)
{
    makeCurrent(); 
    glClearColor(0.7f,0.7f,0.7f,1.0f);   
    updateGL();
}

void MapViewer::focusOutEvent(QFocusEvent *fe)
{
    makeCurrent();  
    //glClearColor(0.3f,0.3f,0.3f,1.0f);
    updateGL(); 
}

QImage MapViewer::captureMap()
{
    return grabFrameBuffer();
}

MapViewer::~MapViewer()
{

}

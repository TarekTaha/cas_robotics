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
	//pixmap = QPixmap("resources/casareaicp.png");
  	clearColor = Qt::black;
    setFocusPolicy(Qt::StrongFocus);
    makeCurrent(); 
    glGenTextures(1, &texId); 
    qWarning("Initialized !!!"); fflush(stdout);	
}

GLuint MapViewer::makeObject()
{
    //glOrtho(-aspectRatio, aspectRatio, -1, 1, -1000, 1000); 
    static const double coords[4][3] = 
    //{{ +aspectRatio, -aspectRatio, -1 }, { -aspectRatio, -aspectRatio, -1 }, 
    //{ -aspectRatio, +aspectRatio, -1 }, { +aspectRatio, +aspectRatio, -1 } };
   {{ +1, -1, -1 }, { -1, -1, -1 }, { -1, +1, -1 }, { +1, +1, -1 } };

    GLuint texture;
    texture = bindTexture(QPixmap(QString("/home/BlackCoder/workspace/CasPlanner/resources/casareaicp.png")),GL_TEXTURE_2D);

    GLuint list = glGenLists(1);
    glNewList(list, GL_COMPILE);
    glBindTexture(GL_TEXTURE_2D, texture);
    glBegin(GL_QUADS);
    for (int j = 0; j < 4; ++j) 
    {
	    glTexCoord2d(j == 0 || j == 3, j == 0 || j == 1);
        glVertex3d(0.2 * coords[j][0], 0.2 * coords[j][1],0.2 * coords[j][2]);
    }
    glEnd();
    glEndList();
    return list;
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
	
	if (!sharedObject)
    	sharedObject = makeObject();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_TEXTURE_2D);
    
    //glClearColor(0.70f, 0.7f, 0.7f, 1.0f);
    // Some stupid crap for QT
    //renderText(0,0,0,""); 
}

void MapViewer::resizeGL(int w, int h)
{
	int side = qMin(w, h);
//    glViewport((w - side) / 2, (h - side) / 2, side, side);
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//    glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0);
//    glMatrixMode(GL_MODELVIEW);
    //qDebug("resizeGL on OGRenderer called");
    screenWidth = w;
    screenHeight = h;
    aspectRatio = ((float) w)/((float) h); 
    glMatrixMode(GL_PROJECTION); 
    glLoadIdentity(); 
    //qDebug("Aspect ratio set to %f", aspectRatio); 
    glOrtho(-aspectRatio, aspectRatio, -1, 1, -1000, 1000); 
    //glOrtho(-0.5, +0.5, +0.5, -0.5, 4.0, 15.0);
    //gluPerspective(60, aspectRatio, 1,1000); 
    glMatrixMode(GL_MODELVIEW); 
    glLoadIdentity();
    //glTranslatef(0,0,-2);
    glViewport((w - side) / 2, (h - side) / 2, side, side);
    //glViewport(0,0,w,h); 
    updateGL();
}
void MapViewer::update()
{
    qDebug("update on OGRenderer called");
//  QVector<QString> robotIds;
//  QVector<QString> patchIds;
//  QVector<RobotLocation *> robotLocations;
//  QVector<orca::SnapPtr> snaps; 
//  makeCurrent();
//  
//  
//  
//  //qDebug("Update called");
//  if(mapManager)
//  {
//      renderText(0.0, 0.0, 0.0,"     "); 
//      robotIds = mapManager->getRobotIds();
//     //Stoopid opengl Hacky crap. 
//     for(int i=0; i < robotIds.size(); i++){
//	 patchIds = mapManager->getPatchIds(robotIds[i]);
//	 for(int j=0; j < patchIds.size(); j++){
//	     snaps = mapManager->getSnaps(robotIds[i], patchIds[j]); 
//	     for(int k=0; k < snaps.size(); k++){
//		 QString snapName = QString(snaps[k]->desc.c_str());
//		 // qDebug("Allocated 2 list %d to %s", snapDLs[snapName], qPrintable(snapName));
//		 if(!snapDLs.contains(snapName)){
//		     GLenum foo = glGetError();
//		     if(foo != GL_NO_ERROR){
//			 qWarning("PostSnap construct 1 error number %d", foo);   
//		     }
//		     snapDLs[snapName] = glGenLists(1); 
//		     qDebug("Allocated list %d to %s", snapDLs[snapName], qPrintable(snapName));
//		     glNewList(snapDLs[snapName], GL_COMPILE);
//		     if(snaps[k]->type == orca::SNAPVICTIM){
//			 glColor4f(1,0,0,1); 
//			 renderText(0.0, 0.0, 0.0, snapName); 
//		     }
//		     else if(snaps[k]->type == orca::SNAPLANDMARK){
//			 glColor4f(0,1,0,1); 
//			 renderText(0.0, 0.0, 0.0, snapName);
//		     }
//		     glEndList();
//		    foo = glGetError();
//		     if(foo != GL_NO_ERROR){
//			 qWarning("PostSnap construct 1 error number %d", foo);   
//		     }
//		 }
//	     }
//	 }
//     }
//     glFlush();
//
//     
//     glNewList(displayList, GL_COMPILE);
//     robotIds = mapManager->getRobotIds();
//    
//    for(int i=0; i < robotIds.size(); i++)
//    {
//	glPushMatrix();
//	MapObject *robotMO = mapManager->getMORobotMap(robotIds[i]);
//	if(robotMO->getVisibility() > 0){
//	float roXfrm[16]; 
//	robotMO->getOrigin().getGLTransform(roXfrm); 
//	glMultMatrixf(roXfrm); 
//	//qDebug("Got robot id: %s", qPrintable(robotIds[i])); 
//      patchIds = mapManager->getPatchIds(robotIds[i]);
//      
//      for(int j=0; j < patchIds.size(); j++)
//      {
//	  
//	  //qDebug("Got patch id: %s", qPrintable(patchIds[j])); 
//	// Each patch is draw relative to the parent, so, though it is counterintiutive,
//	// this is an unmatch translate and rotated.  
//        
//	  GLenum foo = glGetError();
//	  if(foo != GL_NO_ERROR){
//	      qWarning("OpenGL 1 error number %d", foo);   
//	  }
//	//qDebug("Got snaps");
//        float xfrm[16];
//	mapManager->getParentOrigin(robotIds[i],patchIds[j])->getGLTransform(xfrm);
//	//qDebug("Matrix is:");
//	//qDebug("%f %f %f %f", xfrm[0], xfrm[4], xfrm[8], xfrm[12]);
//	//qDebug("%f %f %f %f", xfrm[1], xfrm[5], xfrm[9], xfrm[13]);
//	//qDebug("%f %f %f %f", xfrm[2], xfrm[6], xfrm[10], xfrm[14]);
//	//qDebug("%f %f %f %f", xfrm[3], xfrm[7], xfrm[11], xfrm[15]);
//	//qDebug("Got origin"); 
//          //qDebug("Got patch id: %s", qPrintable(patchIds[j])); 
//	// Each patch is draw relative to the parent, so, though it is counterintiutive,
//	// this is an unmatch translate and rotated.  
//	glMultMatrixf(xfrm);
//	orca::OgMapDataPtr currentMap = mapManager->getOccugrid(robotIds[i], patchIds[j]);
//	if(!currentMap){
//	    qDebug("Current map was null!!"); 
//	}
//	MapObject *patchMO = mapManager->getMOPatch(robotIds[i], patchIds[j]); 
//	if(showOGs && patchMO->getVisibility() == 1){
//	    renderOG(currentMap);
//	}
//	else if(showOGs && patchMO->getVisibility() == 2) {
//	    renderOG(currentMap, true); 
//	}
//	foo = glGetError();
//	if(foo != GL_NO_ERROR){
//	    qWarning("OpenGL 2 error number %d", foo);   
//	}
//	
//
//	QVector<orca::PointcloudPtr> pointclouds =  mapManager->getPointclouds(robotIds[i], patchIds[j]); 
//	for(int k=0; k < pointclouds.size(); k++){
//	    // Horrible disgusting hack, but what's a guy to do when the SR is playing up? 
//	    if(robotIds[i].startsWith("CASTER")){
//		renderPointcloud(pointclouds[k], fudgeFactor);
//	    }
//	    else {
//		renderPointcloud(pointclouds[k]);
//	    }
//	      
//	}
//
//	foo = glGetError();
//	if(foo != GL_NO_ERROR){
//	    qWarning("OpenGL 3 error number %d", foo);   
//	}
//        // Now do victims and Landmarks.
//	//qDebug("Doing %d snaps", snaps.size())
//	if(showSnaps){
//	    snaps = mapManager->getSnaps(robotIds[i], patchIds[j]); 
//	    for(int k=0; k < snaps.size(); k++){
//		if(robotIds[i].startsWith("CASTER")){
//		    renderSnap(snaps[k], fudgeFactor);
//		}
//		else {
//		    renderSnap(snaps[k]);
//		} 
//	    }
//	}
//	foo = glGetError();
//	if(foo != GL_NO_ERROR){
//	    qWarning("OpenGL 4 error number %d", foo);   
//	}
//	//qDebug("Finished victims %d", snaps.size());
//        // Now do Robot Positions.
//	if(showRobots &&  patchMO->getVisibility() != 0){
//	    robotLocations = mapManager->getMORobotLocations(robotIds[i], patchIds[j]);
//	    if(j == (patchIds.size()-1)){
//		renderRobots(robotLocations, true); 
//	    }
//	    else {
//		renderRobots(robotLocations, false); 
//	    }
//	} 
//	foo = glGetError();
//	if(foo != GL_NO_ERROR){
//	    qWarning("OpenGL 5 error number %d", foo);   
//	}
//      }
//	}
//      glPopMatrix(); // This should match the per robot push at the top. 
//	
//    }
//    glEndList();
//  }
//  else
//  {
//    qWarning("Map manager not set!!");
//  }
//  updateGL();
}

void MapViewer::paintGL()
{
	qDebug("paintGL on OGRenderer called");
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    glEnable(GL_BLEND);
//    glDisable(GL_DEPTH_TEST);
//    glEnable(GL_POINT_SMOOTH); 
//    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST); 
//    glEnable(GL_LINE_SMOOTH); 
//    glEnable(GL_POLYGON_SMOOTH);
//    glMatrixMode(GL_MODELVIEW);
//    glPushMatrix();
//    glScalef(1/zoomFactor, 1/zoomFactor, 1/zoomFactor);
//    glColor4f(0,0,0,1); 
//    glBegin(GL_LINES); 
//    glVertex2f(zoomFactor*aspectRatio*0.90-1, -0.9*zoomFactor);
//    glVertex2f(zoomFactor*aspectRatio*0.90, -0.9*zoomFactor); 
//    glEnd(); 
//    renderText(zoomFactor*aspectRatio*0.90-1, -0.9*zoomFactor, 0, "scale: 1 m");
//    glRotatef(pitch,1,0,0); 
//    glRotatef(yaw,0,0,1); 
// 
//    glTranslatef(xOffset, yOffset, zOffset);
//
//    if(showGrids){
//	for(int i=-(int) zoomFactor*3; i < (int) zoomFactor*3; i++){
//	    glBegin(GL_LINES);
//	    if(i==0){
//		glColor4f(0,0,0,0.5);  
//	    }
//	    else {
//		glColor4f(0.5,0.5,0.5,0.5); 
//	    }
//	    glVertex3f(-zoomFactor*3, i, 0); 
//	    glVertex3f(zoomFactor*3, i, 0); 
//	    glVertex3f(i,-zoomFactor*3, 0); 
//	    glVertex3f(i, zoomFactor*3, 0); 
//	    glEnd(); 
//	}
//    }
//    //Ok, now let's draw dem maps.
//    glColor4f(0,0,0,1.0);
//    glPushMatrix();  
//    glPopMatrix(); 
//    
//    glDisable(GL_BLEND);
//    glEnable(GL_DEPTH_TEST);
//    glDisable(GL_POINT_SMOOTH); 
//    glDisable(GL_LINE_SMOOTH); 
//    glDisable(GL_POLYGON_SMOOTH);
//    glPopMatrix();
    
    qglClearColor(clearColor);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glTranslated(0.0, 0.0, -10.0);
    glRotated(90.0 / 16.0, 1.0, 0.0, 0.0);
    glRotated(25.0 / 16.0, 0.0, 1.0, 0.0);
    glRotated(45.0 / 16.0, 0.0, 0.0, 1.0);
    glCallList(sharedObject);    

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

void MapViewer::setShowSnaps(int state){
    if(state==0){
	showSnaps = false;  
    }
    else {
	showSnaps = true; 
    }
    update();
}
void MapViewer::setShowGrids(int state){
    if(state==0){
	showGrids = false;  
    }
    else {
	showGrids = true; 
    }
    update();
}

void MapViewer::setShowRobots(int state){
    if(state==0){
	showRobots = false;  
    }
    else {
	showRobots = true; 
    }
    update();
}	
	
void MapViewer::setShowPointclouds(int state){
    if(state==0){
	showPointclouds = false;  
    }
    else {
	showPointclouds = true; 
    }
    update(); 
}

void MapViewer::setShowPatchBorders(int state){
    if(state==0){
	showPatchBorders = false;  
    }
    else {
	showPatchBorders = true; 
    }
    update(); 
}
	
//void MapViewer::renderPointcloud(orca::PointcloudPtr pointcloud, float in_fudgeFactor){
//    //qDebug("About to do pointclouds 2... %d %d", pointclouds.size(), &pointcloud);
//	     
//
//	    //qDebug("Doing pointcloud %d which has points at address %d", k, pointcloud->points.size());
//    if(showPointclouds){
//    glPushMatrix();
//	    //qDebug("Translating to: %f %f %f", pointcloud->sensorPose.p.x, pointcloud->sensorPose.p.y, pointcloud->sensorPose.p.z);
//	    //glTranslatef(pointcloud->sensorPose.p.x, pointcloud->sensorPose.p.y, pointcloud->sensorPose.p.z);
//    Vector6DOF robotPose = mapManager->convertFrame3dToVector6DOF(pointcloud->robotPose);
//    float rpMatrix[16]; 
//    robotPose.getGLTransform(rpMatrix);
//    glMultMatrixf(rpMatrix);  
//    Vector6DOF sensorPose = mapManager->convertFrame3dToVector6DOF(pointcloud->sensorPose);
//    float spMatrix[16];
//    sensorPose.getGLTransform(spMatrix);
//    glMultMatrixf(spMatrix);  
//    GLenum foo = glGetError();
//    if(foo != GL_NO_ERROR){
//	qWarning("OpenGL error at start: %d", foo);   
//    }
//    glScalef(0.01,0.01,0.01); // Raymond says units are cm. 
//	    
//	    //glBlendColor(1,1,1,0.5); 
//	    //glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA); 
//    glScalef(in_fudgeFactor, in_fudgeFactor, in_fudgeFactor); 
//    glDisable(GL_POINT_SMOOTH);
//    glPointSize(screenWidth/zoomFactor*0.01);
//	    //glRotatef(-90,0,0,1); 
//    glBegin(GL_POINTS);
//    for(int l=0; l < pointcloud->points.size()/6; l++){
//	glColor4f(pointcloud->points[l*6+3], pointcloud->points[l*6+4], pointcloud->points[l*6+5], 0.3); 
//	glVertex3f(pointcloud->points[l*6], pointcloud->points[l*6+1], pointcloud->points[l*6+2]); 
//		//qDebug("z value is: %f", pointcloud->points[l*6+2]);
//		//glVertex3f(pointcloud->points[l*6]/pointcloud->points[l*6+2]*-100, pointcloud->points[l*6+1]/pointcloud->points[l*6+2]*-100, -100);
//		
//    }
//    glEnd();
//    glPointSize(1.0); 
//	    
//	    /*
//    glColor4f(0,0,1,0.5); 
//    glBegin(GL_QUADS);
//    glVertex3f(-50,-50,-100);
//    glVertex3f(-50,50,-100); 
//    glVertex3f(50,50,-100); 
//    glVertex3f(50,-50,-100);
//    glEnd();
//	    */
//
//    glPopMatrix();
//    }
//}
//
//void MapViewer::renderSnap(orca::SnapPtr snap, float in_fudgeFactor){
//    //Render pointcloud first.
//    orca::PointcloudPtr ptc = new orca::Pointcloud(snap->robotPose, snap->sensorPose, snap->cloud); 
//    renderPointcloud(ptc, in_fudgeFactor); 
//    glPushMatrix();
//    Vector6DOF robotPose = mapManager->convertFrame3dToVector6DOF(snap->robotPose);
//    float rpMatrix[16]; 
//    robotPose.getGLTransform(rpMatrix);
//    glMultMatrixf(rpMatrix);  
//    Vector6DOF sensorPose = mapManager->convertFrame3dToVector6DOF(snap->sensorPose);
//    float spMatrix[16];
//    sensorPose.getGLTransform(spMatrix);
//    glMultMatrixf(spMatrix);  
//    glScalef(0.01,0.01,0.01); // Raymond says units are cm. 
//    
//	    //glBlendColor(1,1,1,0.5); 
//	    //glBlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA); 
//    glScalef(in_fudgeFactor, in_fudgeFactor, in_fudgeFactor); 
//    glPointSize(10); 
//    int foo = glGetError(); 
//    if(foo != GL_NO_ERROR){ 
//	qWarning("OpenGL error number BEFORE snap %d", foo);   
//    }
//    if(snap->type == orca::SNAPVICTIM){
//	glColor4f(1,0,0,1); 
//	glBegin(GL_POINTS); 
//	glVertex3f(snap->objectPosition.x, snap->objectPosition.y, snap->objectPosition.z); 
//	glEnd();
//	glPushMatrix();
//	glTranslatef(snap->objectPosition.x, snap->objectPosition.y, snap->objectPosition.z); 
//	int DL = snapDLs[QString(snap->desc.c_str())]; 
//	// qDebug("Using dl %d", DL); 
//	glCallList(DL); 
//	glPopMatrix(); 
//    }
//    else if(snap->type == orca::SNAPLANDMARK){
//	glColor4f(0,1,0,1); 
//	glBegin(GL_POINTS); 
//	glVertex3f(snap->objectPosition.x, snap->objectPosition.y, snap->objectPosition.z); 
//	glEnd();
//	glPushMatrix();
//	glTranslatef(snap->objectPosition.x, snap->objectPosition.y, snap->objectPosition.z); 
//	int DL = snapDLs[QString(snap->desc.c_str())]; 
//	// qDebug("Using dl %d", DL); 
//	glCallList(DL); 
//	glPopMatrix(); 
//    }
//    else {
//	qWarning("Unknown snap type %d", snap->type); 
//    }
//    glFlush();
//    foo = glGetError();
//    if(foo != GL_NO_ERROR){
//	qWarning("OpenGL error number AFTER snap %d", foo);   
//    }
//    glPopMatrix();
//}

//void MapViewer::renderRobots(QVector<RobotLocation *> robotLocations, bool renderRobot)
//{
//    glColor4f(0,0,0,1);
//    glBegin(GL_LINE_STRIP);
//    if(robotLocations.size()> 2){
//	for(int k=2; k < robotLocations.size(); k++)
//	{
//          //qDebug("Adding location %f %f", robotLocations[j]->position.getX(), -robotLocations[j]->position.getY());
//	    glVertex2f(robotLocations[k]->position.getX(), robotLocations[k]->position.getY());
//	}
//    }
//    glEnd();
//    if(!robotLocations.isEmpty()){
//	glPushMatrix(); 
//	RobotLocation *r = robotLocations.last();
//	    //qDebug("Got RobotLocations"); 
//	glTranslatef(r->position.getX(), r->position.getY(),0); 
//	    //qDebug("Last rotation is: %f", r->position.getYawDeg());
//	glRotatef(r->position.getYawDeg(), 0,0,1); 
//	    //qDebug("Got Last");
//	glBegin(GL_TRIANGLES);
//	glVertex2f(0,0.1);
//	glVertex2f(0,-0.1); 
//	glVertex2f(0.3,0); 
//	glEnd();
//	glPopMatrix();
//    } 
//}
//
//void MapViewer::renderOG(orca::OgMapDataPtr currentMap, bool highlighted)
//{
//    int textWidth;
//    int textHeight; 
//    float textWFrac; 
//    float textHFrac; 
// 
//#ifdef NOPOTD
//    // Find the next biggest power of two. 
//   textWidth = powl(2, ceill(log(currentMap->numCellsX)/log(2)));
//   textHeight = powl(2, ceill(log(currentMap->numCellsY)/log(2)));
//   textWFrac = ((float) currentMap->numCellsX)/textWidth; 
//   textHFrac = ((float) currentMap->numCellsY)/textHeight; 
//   qDebug("Width and height are %d and %d", textWidth, textHeight); 
//#else 
//    textWidth = currentMap->numCellsX;
//    textHeight = currentMap->numCellsY;
//    textWFrac = 1; 
//    textHFrac = 1; 
//#endif
//    unsigned char imgData[textWidth*textHeight*4];
//    int min=255; 
//    int max=0; 
//    float mean=0; 
//
//    
//    for(int i=0; i < currentMap->numCellsY; i++)
//    {
//	for(int j=0; j < currentMap->numCellsX; j++)
//	{
//	    // qDebug("Pixel %d has value %d", k, (unsigned char) currentMap->data[k]);
//	    unsigned char val = (unsigned char) currentMap->data[i*currentMap->numCellsX+j];
//	    if(val < min) min=val;
//	    if(val > max) max=val;
//	    mean += val;
//	    if(val > 90){
//		imgData[(i*textWidth+j)*4] = 0;
//		imgData[(i*textWidth+j)*4+1] =0; 
//		imgData[(i*textWidth+j)*4+2] = 255; 
//		imgData[(i*textWidth+j)*4+3] = 255;
//	    }
//	    else if(val < 70){
//		if(!highlighted){
//		    imgData[(i*textWidth+j)*4] = 255;
//		    imgData[(i*textWidth+j)*4+1] =255; 
//		    imgData[(i*textWidth+j)*4+2] = 255; 
//		    imgData[(i*textWidth+j)*4+3] = 50;
//		}
//		else {
//		    imgData[(i*textWidth+j)*4] = 255;
//		    imgData[(i*textWidth+j)*4+1] =255; 
//		    imgData[(i*textWidth+j)*4+2] = 0; 
//		    imgData[(i*textWidth+j)*4+3] = 100;
//		}
//	    }
//	    else {
//		imgData[(i*textWidth+j)*4] = 0;  
//		imgData[(i*textWidth+j)*4+1] = 0; 
//		imgData[(i*textWidth+j)*4+2] = 0;  
//		imgData[(i*textWidth+j)*4+3] = 0;   
//	    }
//	}
//    }
//    qDebug("Map values %d %d %f" , min, max, mean/(currentMap->numCellsX*currentMap->numCellsY)); 
//
//    glBindTexture(GL_TEXTURE_2D, texId); 
//    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, textWidth, textHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, imgData);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
//		    GL_LINEAR);
//    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
//		    GL_LINEAR);
//    glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
//    glEnable(GL_TEXTURE_2D);
//    glPushMatrix();
//    glScalef(1,-1,1);
//    glTranslatef(-currentMap->numCellsX*currentMap->metresPerCellX/2.0,-currentMap->numCellsY*currentMap->metresPerCellY/2,0);
//    glColor4f(0,0,1,0.8);
//    glShadeModel(GL_FLAT);
//    glBegin(GL_QUADS);
//    
//    glTexCoord2f(0.0,0.0);
//    glVertex2f(0.0,0.0);
//    glTexCoord2f(0.0,textHFrac);
//    glVertex2f(0.0, currentMap->numCellsY*currentMap->metresPerCellY);
//    glTexCoord2f(textWFrac, textHFrac);
//    glVertex2f(currentMap->numCellsX*currentMap->metresPerCellX,currentMap->numCellsY*currentMap->metresPerCellY);
//    glTexCoord2f(textWFrac,0.0);
//    glVertex2f(currentMap->numCellsX*currentMap->metresPerCellX,0.0);
//    glEnd();
//    glDisable(GL_TEXTURE_2D);
//    
//    // We now draw a surrounding rectangle so people know where the edge of the OG map is. 
//    if(showPatchBorders){
//	glColor4f(0,0,1,0.5); 
//	glBegin(GL_LINE_LOOP); 
//	glVertex2f(0,0);
//	glVertex2f(0.0, currentMap->numCellsY*currentMap->metresPerCellY);
//	glVertex2f(currentMap->numCellsX*currentMap->metresPerCellX,currentMap->numCellsY*currentMap->metresPerCellY);
//	glVertex2f(currentMap->numCellsX*currentMap->metresPerCellX,0.0);
//	glEnd();
//    } 
//    glPopMatrix();
//    
//}

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
    update(); 
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

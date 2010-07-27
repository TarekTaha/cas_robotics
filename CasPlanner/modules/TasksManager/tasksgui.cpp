/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
 *                                                                         *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "tasksgui.h"
#include "logger.h"

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
	testModelBtn("Test Bayesian Model"),
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
    actionLayout->addWidget(&testModelBtn);
    actionLayout->addWidget(&pauseBtn);
    actionLayout->addWidget(&captureImage);
    actionLayout->addWidget(&generateSkeletonBtn);
    actionGB.setLayout(actionLayout);

    connect(&innerSkeletonBtn, SIGNAL(toggled(bool )), this,SLOT(updateSelectedVoronoiMethod(bool)));
    connect(&outerSkeletonBtn, SIGNAL(toggled(bool )), this,SLOT(updateSelectedVoronoiMethod(bool)));
    connect(&generateSkeletonBtn, SIGNAL(pressed()),tasksGui, SLOT(generateSkeleton()));
    connect(&captureImage,     SIGNAL(pressed()),this, SLOT(save()));
    connect(&randomTasksBtn,   SIGNAL(pressed()),this, SLOT(runRandomTasks()));
    connect(&tasksList,        SIGNAL(currentRowChanged(int)),this, SLOT(taskSelected(int)));
    connect(&testModelBtn,     SIGNAL(pressed()),tasksGui, SLOT(testModel()));
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
    int sourceVertixId,destVertexId;
    t.start();
    for(int i=0;i<numRandomRuns.value();i++)
    {
        // find a random source state, any stata can be used
        sourceVertixId = rand()%tasksGui->playGround->mapManager->mapSkeleton.getNumVerticies();
        // chose randomly one of the destinations
        destVertexId   = rand()%tasksGui->playGround->mapManager->mapSkeleton.getNumDestinations();
        // find the destination's actual index
        destVertexId = tasksGui->playGround->mapManager->mapSkeleton.destIndexes[destVertexId];
        qDebug()<<"\n Source Vertex is:"<<sourceVertixId<<" destination Vertex is:"<<destVertexId;
        //tasksList.setCurrentRow(r);
        //qDebug("Using Task %s %d ",qPrintable(tasksGui->tasks[sourceVertixId].getName()),sourceVertixId);
        Pose start(tasksGui->playGround->mapManager->mapSkeleton.verticies[sourceVertixId].getLocation().x(),
                   tasksGui->playGround->mapManager->mapSkeleton.verticies[sourceVertixId].getLocation().y(),
                   0);
        Pose end (tasksGui->playGround->mapManager->mapSkeleton.verticies[destVertexId].getLocation().x(),
                   tasksGui->playGround->mapManager->mapSkeleton.verticies[destVertexId].getLocation().y(),
                   0);
        tasksGui->voronoiPlanner->startSearch(start,end,METRIC);
        int step=0;
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
                qDebug()<<"Set:"<<step++<<" has index:"<<tasksGui->playGround->mapManager->mapSkeleton.getCurrentSpatialState(p->pose);
                p = p->next;
            }
        }
    }
    qDebug()<<QString("Generationg %1 Random Paths took:%2").arg(numRandomRuns.value()).arg(t.elapsed());
}

void TasksControlPanel::updateSelectedVoronoiMethod(bool)
{

}

void TasksControlPanel::save()
{

}

void TasksControlPanel::taskSelected(int r)
{
    //qDebug()<<"Selected Task is:"<<r;
    Pose start(tasksGui->tasks[r].getStart().x(),tasksGui->tasks[r].getStart().y(),0);
    Pose   end(tasksGui->tasks[r].getEnd().x(),tasksGui->tasks[r].getEnd().y(),0);
    if(tasksGui->voronoiPlanner)
        tasksGui->voronoiPlanner->startSearch(start,end,METRIC);
    fflush(stdout);
}

void TasksControlPanel::setMap(QImage)
{

}

void TasksControlPanel::exportHtml()
{

}

MapGL::MapGL(Map*og,TasksGui *tsg, QWidget *parent):
	QGLWidget(QGLFormat(QGL::AlphaChannel), parent),
	tasksGui(tsg),
	zoomFactor(10),
	xOffset(0),
	yOffset(0),
	zOffset(0),
	yaw(0),
	pitch(0),
	fudgeFactor(3),
	showGrids(true),
	firstTime(true),
	mainMapBuilt(false),
	mapSkeleton(NULL),
	ogMap(og)
{
    renderTimer = new QTimer(this);
    connect(renderTimer, SIGNAL(timeout()), this, SLOT(updateGL()));
    renderTimer->start(200);
    setFocusPolicy(Qt::StrongFocus);
}

QSize MapGL::sizeHint()
{
    return QSize(800,800);
}

QSize MapGL::setMinimumSizeHint()
{
    return QSize(400,400);
}

void MapGL::initializeGL()
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glFlush();
    skeletonList = glGenLists(1);
    mapList = glGenLists(1);
}

void MapGL::resizeGL(int w, int h)
{
    aspectRatio = ((float) w)/((float) h);
    // Reset the coordinate system before modifying
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
}

void MapGL::setMapSkeleton(MapSkeleton *mapS)
{
    this->mapSkeleton = mapS;
}

void MapGL::config()
{

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
    if(!mapSkeleton)
        return;
    glPushMatrix();
    glLineWidth(2);

    for(int i=0;i<mapSkeleton->getNumVerticies();i++)
    {
        glColor4f(0,1,0,1);;
        QPointF parentVertex = mapSkeleton->verticies[i].getLocation();
        for(int j=0;j<mapSkeleton->verticies[i].connections.size();j++)
        {
            QPointF child = mapSkeleton->verticies[mapSkeleton->verticies[i].connections[j].nodeIndex].getLocation();
            glBegin(GL_LINES);
                glVertex2f(parentVertex.x(),parentVertex.y());
                glVertex2f(child.x(),child.y());
            glEnd();
        }
        if(mapSkeleton->destIndexes.contains(i))
        {
            glBegin(GL_POLYGON);
                glColor4f(1,0,0,1);
                glVertex2f(parentVertex.x()- 0.2, parentVertex.y()+0.2);
                glVertex2f(parentVertex.x()+ 0.2, parentVertex.y()+0.2);
                glVertex2f(parentVertex.x()+ 0.2, parentVertex.y()-0.2);
                glVertex2f(parentVertex.x()- 0.2, parentVertex.y()-0.2);
            glEnd();
        }
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
}

void MapGL::loadTexture()
{
    newWidth =  (int) std::pow(2.0f, (int)ceil(log((float)ogMap->width) / log(2.f)));
    newHeight = (int) std::pow(2.0f, (int)ceil(log((float)ogMap->height) / log(2.f)));
    ratioW  = ((float) ogMap->width)/newWidth;
    ratioH  = ((float) ogMap->height)/newHeight;;
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
                imgData[(j*ogMap->width+i)*4+3] = 50;
            }
        }
    }

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

/*!
 *  Renders The main Map loaded from the image file
 */
void MapGL::renderMap()
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
    glColor4f(0,1.0,0,1.0);
    glBegin(GL_LINE_LOOP);
    glVertex2f(0,0);
    glVertex2f(0.0,newHeight*ogMap->mapRes);
    glVertex2f(newWidth*ogMap->mapRes,newHeight*ogMap->mapRes);
    glVertex2f(newWidth*ogMap->mapRes,0.0);
    glEnd();
    glPopMatrix();
    glDisable(GL_TEXTURE_2D);
    glEndList();
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
        glEndList();
        glCallList(skeletonList);
        firstTime = false;
    }
    else
        glCallList(skeletonList);
    if(this->ogMap)
    {
        glDisable( GL_DEPTH_TEST );
        if(!mainMapBuilt)
        {
            loadTexture();
            renderMap();
        }
        glEnable(GL_DEPTH_TEST);
    }
    glCallList(mapList);
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
    }
    else if(e->key() == Qt::Key_BracketRight)
    {
        zoomFactor /= 1.1;
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
    }
    else if(e->text()=="-")
    {
        fudgeFactor /=1.25;
    }
    else if(e->text() == "0")
    {
        fudgeFactor=3;
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
	mapGL(playG->mapManager->globalMap,this,parent),
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
// 	std::ifstream in(filename.c_str());
// 	tasks.clear();
// 	int row=0;
//	if ( in )
//    {
//    	while ( ! in.eof() )
//      	{
//          	double x1,x2,y1,y2 ;
//          	string name;
//          	//char name[20];
//          	in >> x1 >> y1 >> x2 >> y2>> name ;
//          	Task t(QPointF(x1,y1),QPointF(x2,y2),name.c_str());
//	        tasks.push_back(t);
//
//	        QListWidgetItem *newItem = new QListWidgetItem;
//	        newItem->setText(name.c_str());
//	        tasksControlPanel.tasksList.insertItem(row++, newItem);
//      	}
//    }
//	else
//	{
//		std::cout<<"\n File Not Found";
//	}
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


TasksGui::~TasksGui()
{
}

void TasksGui::resetTab()
{
    // get this tab index
    int index = tabContainer->indexOf(this);
    // set tab title
    tabContainer->setTabText(index, "Cas Planner");
}

void TasksGui::requestSnap()
{

}

void TasksGui::testModel()
{
//  const std::string pomdpFileName = "/home/BlackCoder/Desktop/paperexperiment.pomdp";
//  // read it in
//  zmdp::Pomdp p(pomdpFileName);
//
//  // print out stats
//  cout << "numStates = " << p.getBeliefSize() << endl;
//  cout << "numActions = " << p.getNumActions() << endl;
//  cout << "numObservations = " << p.getNumObservations() << endl;
//  cout << "discount = " << p.getDiscount() << endl;
//  cout << endl;

  // seeds random number generator
  MatrixUtils::init_matrix_utils();

  ZMDPConfig* config = new ZMDPConfig();
  config->readFromFile("/home/BlackCoder/Desktop/something.conf");
  config->setString("policyOutputFile", "none");
  BoundPairExec* em = new BoundPairExec();
  printf("initializing\n");

  em->initReadFiles("/home/BlackCoder/Desktop/paperexperiment.pomdp","/home/BlackCoder/Desktop/out.policy", *config);

//  MDPExec* e = em;
  printf("Number of Actions is:%d\n",em->mdp->getNumActions());
//  belief_vector b(((Pomdp*)em->mdp)->getBeliefSize());
  belief_vector b;
  dvector initialBeliefD;
  initialBeliefD.resize(((Pomdp*)em->mdp)->getBeliefSize());
  //belief_vector b = ((Pomdp*)em->mdp)->getInitialBelief();
//  initialBeliefD(5)=1;
//  copy(b, initialBeliefD);
//  for(int i=0; i < b.size();i++)
//  {
//  	if(b(i))
//  	{
//	  	printf("\nBelief%d=%f",i,b(i));
//	  	fflush(stdout);
//  	}
//  }
//  em->setBelief(b);

//  printf("  reset to initial belief\n");
    em->setToInitialState();
//  belief_vector newB = em->currentState;
//  for(int i=0; i < newB.size();i++)
//  {
//  	if(newB(i))
//  	{
//	  	printf("\nBelief%d=%f",i,newB(i));
//	  	fflush(stdout);
//  	}
//  }
  int obs[]={2,2,2,2,2,2,2,2,2,2,2,2,2,0,4};
  int NUM_TRIALS = 15 ;
  for (int i=0; i < NUM_TRIALS; i++)
  {
            printf("  step %d\n", i);
            int a = em->chooseAction();
            printf("    chose action %d\n", a);
            int o = obs[i];//em->getRandomOutcome(a);
            em->advanceToNextState(a,o);
            printf("    updated belief\n");
            belief_vector newB = em->currentState;
            double max = 0;
            int index=0;
                for(unsigned int j=0; j < newB.size();j++)
                {
                        if(newB(j)&& newB(j)>max)
                        {
                                max=newB(j);
                                index=j;
                        }
                }
                printf("\nNew Belief: %d=%f",index,max);
                fflush(stdout);
                if (em->getStateIsTerminal())
            {
                        printf("  [belief is terminal, ending trial]\n");
                        break;
            }
    }
//  int s, sp, a, o;
//
//  printf("R(s,a) matrix (%d x %d) =\n", p.R.size1(), p.R.size2());
//  for (s=0; s < p.getBeliefSize(); s++) {
//    for (a=0; a < p.getNumActions(); a++) {
//      printf("%9.2f ", p.R(s,a));
//    }
//    cout << endl;
//  }
//  cout << endl;
//
//  for (a=0; a < p.getNumActions(); a++) {
//    printf("T_%d(s,sp) matrix (%d x %d) =\n", a, p.T[a].size1(), p.T[a].size2());
//    for (s=0; s < p.getBeliefSize(); s++) {
//      for (sp=0; sp < p.getBeliefSize(); sp++) {
//	printf("%5.3f ", p.T[a](s,sp));
//      }
//      cout << endl;
//    }
//    cout << endl;
//  }
//
//  for (a=0; a < p.getNumActions(); a++) {
//    printf("Ttr_%d(sp,s) matrix (%d x %d) =\n", a, p.Ttr[a].size1(), p.Ttr[a].size2());
//    for (sp=0; sp < p.getBeliefSize(); sp++) {
//      for (s=0; s < p.getBeliefSize(); s++) {
//	printf("%5.3f ", p.Ttr[a](sp,s));
//      }
//      cout << endl;
//    }
//    cout << endl;
//  }
//
//  for (a=0; a < p.getNumActions(); a++) {
//    printf("O_%d(sp,o) matrix (%d x %d) =\n", a, p.O[a].size1(), p.O[a].size2());
//    for (sp=0; sp < p.getBeliefSize(); sp++) {
//      for (o=0; o < p.getNumObservations(); o++) {
//	printf("%5.3f ", p.O[a](sp,o));
//      }
//      cout << endl;
//    }
//    cout << endl;
//  }
}

void TasksGui::generateSkeleton()
{
    if(!playGround->mapManager->skeletonGenerated)
    {
        playGround->mapManager->generateSkeleton();
        if(playGround->mapManager->mapSkeleton.numStates !=0)
        {
            skeletonGenerated = true;
            mapGL.paintGL();
            if (voronoiPlanner)
            {
                delete voronoiPlanner;
            }
            voronoiPlanner = new VoronoiPathPlanner(playGround->mapManager->mapSkeleton);
            voronoiPlanner->setMap(playGround->mapManager->globalMap);
            voronoiPlanner->setRobot(playGround->robotPlatforms[0]->robot);
            mapGL.setMapSkeleton(&playGround->mapManager->mapSkeleton);
        }
    }
}

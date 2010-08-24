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
	clearAllBtn("Clear All"),
	saveData("Save Data"),
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
    QHBoxLayout *parHLayout2 = new QHBoxLayout;
    QHBoxLayout *parHLayout3 = new QHBoxLayout;
    QVBoxLayout *parVLayout = new QVBoxLayout;
    parHLayout->addWidget(new QLabel("Random Runs"));
    parHLayout->addWidget(&numRandomRuns);
    parHLayout2->addWidget(new QLabel("Time of Day"));
    parHLayout2->addWidget(&timeOfDay);
    parHLayout3->addWidget(&saveData);

    parVLayout->addLayout(parHLayout);
    parVLayout->addLayout(parHLayout2);
    parVLayout->addLayout(parHLayout3);
    parVLayout->addWidget(&randomTasksBtn);
    parVLayout->addWidget(&clearAllBtn);
    randomTasksGB.setLayout(parVLayout);

    numRandomRuns.setMinimum(1);
    numRandomRuns.setMaximum(1000);
    numRandomRuns.setSingleStep(1);
    numRandomRuns.setValue(10);

    timeOfDay.addItem("Early Morning");
    timeOfDay.addItem("Morning");
    timeOfDay.addItem("Afternoon");
    timeOfDay.addItem("Night");

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
    connect(&captureImage,     SIGNAL(pressed()),tasksGui, SLOT(saveImage()));
    connect(&randomTasksBtn,   SIGNAL(pressed()),this, SLOT(runRandomTasks()));
    connect(&tasksList,        SIGNAL(currentRowChanged(int)),this, SLOT(taskSelected(int)));
    connect(&testModelBtn,     SIGNAL(pressed()),tasksGui, SLOT(testModel()));
    connect(&clearAllBtn,      SIGNAL(released()),tasksGui,SLOT(clearAll()));
    connect(&timeOfDay,SIGNAL(currentIndexChanged(QString)),tasksGui,SLOT(timeOfDayChanged(QString)));
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
    tasksGui->totalVisits = 0;
    tasksGui->playGround->mapManager->mapSkeleton.resetSegmentVisits();
    tasksGui->playGround->mapManager->mapSkeleton.resetVertexVisits();
    t.start();
    // store the state here so that if it changes during data generation
    // we wont end up with open file handler
    bool saveData2File = saveData.isChecked();
    LOG(Logger::Info,"Save 2 Data is:"<<saveData2File)
    QTextStream stream,mismaskStream;
    QFile file,mismaskFile;
    if(saveData2File)
    {
        QString timeCode  =  QDateTime().currentDateTime().toString("hh:mmAP-dd-MM-yyyy");
        file.setFileName(QString("logs/simulatedData%1.txt").arg(timeCode));
        mismaskFile.setFileName(QString("logs/mismaskSimulatedData%1.txt").arg(timeCode));
        if (!file.open(QIODevice::ReadWrite | QIODevice::Text))
        {
            LOG(Logger::Critical,"Can't create file:"<<file.fileName())
            return;
        }
        if (!mismaskFile.open(QIODevice::ReadWrite | QIODevice::Text))
        {
            LOG(Logger::Critical,"Can't create mismaskFile:"<<mismaskFile.fileName())
            return;
        }
        stream.setDevice(&file);
        mismaskStream.setDevice(&mismaskFile);
    }
    for(int i=0;i<numRandomRuns.value();i++)
    {
        // find a random source state, any stata can be used
        sourceVertixId = rand()%tasksGui->playGround->mapManager->mapSkeleton.getNumVerticies();
        // chose randomly one of the destinations
        destVertexId   = rand()%tasksGui->playGround->mapManager->mapSkeleton.getNumDestinations();
        // find the destination's actual index
        destVertexId = tasksGui->playGround->mapManager->mapSkeleton.destIndexes[destVertexId];
        int destIndex = tasksGui->playGround->mapManager->mapSkeleton.getDestIndex(destVertexId);
        LOG(Logger::Info,"\n Source Vertex is:"<<sourceVertixId<<" destination Vertex is:"<<destVertexId)
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
            while(p && p->next)
            {
                int j = tasksGui->playGround->mapManager->mapSkeleton.getVertexWithLocation(p->pose.p);
                int k = tasksGui->playGround->mapManager->mapSkeleton.getVertexWithLocation(p->next->pose.p);
                // Increment visits to this path segment
                tasksGui->playGround->mapManager->mapSkeleton.incrementConnectionVisitsBiDir(j,k);
                //Last Node Pairs
                if(!p->next->next)
                {
                    // Increment the number of visits to each Vertex
                    tasksGui->playGround->mapManager->mapSkeleton.verticies[j].incrementVisits(1);
                    tasksGui->playGround->mapManager->mapSkeleton.verticies[k].incrementVisits(1);
                    LOG(Logger::Info,"Step:"<<step++<<" has index:"<<tasksGui->playGround->mapManager->mapSkeleton.getCurrentSpatialState(p->pose))
                    LOG(Logger::Info,"Step:"<<step++<<" has index:"<<tasksGui->playGround->mapManager->mapSkeleton.getCurrentSpatialState(p->next->pose))
                    if(saveData2File)
                    {
                        //vec(dest1,joy1,loc1,time1),
                        int obs = tasksGui->playGround->mapManager->mapSkeleton.getConnectionDirection(j,k);
                        stream<<destIndex<<" "<<obs<<" "<<j<<" "<<timeOfDay.currentIndex()<<"\n";
                        mismaskStream<<"1 0 0 0\n";
                        // 4 means stop for the last step
                        stream<<destIndex<<" "<<4<<" "<<k<<" "<<timeOfDay.currentIndex()<<"\n";
                        mismaskStream<<"1 0 0 0\n";
                    }
                }
                else
                {
                    // only increment those verticies that are not part of the destinations otherwise
                    // the probabilities will be wrong !!!
                    // TODO:: think about this
                    if(!tasksGui->playGround->mapManager->mapSkeleton.destIndexes.contains(j))
                        tasksGui->playGround->mapManager->mapSkeleton.verticies[j].incrementVisits(1);
                    LOG(Logger::Info,"Step:"<<step++<<" has index:"<<tasksGui->playGround->mapManager->mapSkeleton.getCurrentSpatialState(p->pose))
                    if(saveData2File)
                    {
                        int obs = tasksGui->playGround->mapManager->mapSkeleton.getConnectionDirection(j,k);
                        stream<<destIndex<<" "<<obs<<" "<<j<<" "<<timeOfDay.currentIndex()<<"\n";
                        mismaskStream<<"1 0 0 0\n";
                    }
                }
                p = p->next;
            }
        }
        if(saveData2File && (i+1)<numRandomRuns.value())
        {
            stream<<"\n";
            mismaskStream<<"\n";
        }
    }
    for(int i=0;i<tasksGui->playGround->mapManager->mapSkeleton.destIndexes.size();i++ )
    {
        int vertexIndex =tasksGui->playGround->mapManager->mapSkeleton.destIndexes[i];
        double prob = tasksGui->playGround->mapManager->mapSkeleton.verticies[vertexIndex].getNumVisits()/double(numRandomRuns.value());
        tasksGui->playGround->mapManager->mapSkeleton.verticies[vertexIndex].setProb(prob);
    }
    LOG(Logger::Info,QString("Generating %1 Random Paths took:%2 msec").arg(numRandomRuns.value()).arg(t.elapsed()))
    (tasksGui->getMapGL()).setShowPaths(false);
    if(saveData2File)
    {
        stream.flush();
        file.close();
        mismaskStream.flush();
        mismaskFile.close();
    }
}

void TasksControlPanel::updateSelectedVoronoiMethod(bool)
{

}

void TasksControlPanel::save()
{

}

void TasksControlPanel::taskSelected(int r)
{
    LOG(Logger::Info,"Selected Task is:"<<r)
    if(!tasksGui->skeletonGenerated)
        tasksGui->generateSkeleton();
    int startIndex = tasksGui->tasks[r].getStartVertex();
    int endIndex   = tasksGui->tasks[r].getEndVertex();
    Pose start(tasksGui->playGround->mapManager->mapSkeleton.verticies[startIndex].getLocation().x(),tasksGui->playGround->mapManager->mapSkeleton.verticies[startIndex].getLocation().y(), 0);
    Pose end(tasksGui->playGround->mapManager->mapSkeleton.verticies[endIndex].getLocation().x(),tasksGui->playGround->mapManager->mapSkeleton.verticies[endIndex].getLocation().y(), 0);
    if(tasksGui->voronoiPlanner)
        tasksGui->voronoiPlanner->startSearch(start,end,METRIC);
    tasksGui->getMapGL().setShowPaths(true);
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
	zoomFactor(15),
	xOffset(0),
	yOffset(0),
	zOffset(0),
	yaw(0),
	pitch(0),
	fudgeFactor(3),
	showGrids(true),
	firstTime(true),
	mainMapBuilt(false),
	showPaths(false),
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
    morningList  = glGenLists(1);
    earlyMorningList = glGenLists(1);
    afternoonList = glGenLists(1);
    nightList = glGenLists(1);
    mapList = glGenLists(1);
}

void MapGL::resizeGL(int w, int h)
{
    aspectRatio = ((float) w)/((float) h);
    // Reset the coordinate system before modifying
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, aspectRatio, -5,1000);
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

void MapGL::wheelEvent( QWheelEvent * event )
{
    int numDegrees = event->delta() / 8;
    int numSteps = numDegrees / 15;
    if(numSteps > 0)
        zoomFactor /= 1.1;
    else
        zoomFactor *= 1.1;
}

void MapGL::mousePressEvent(QMouseEvent *me)
{
}

void MapGL::mouseMoveEvent(QMouseEvent *me)
{
}

void MapGL::drawProbHisto(QPointF pos, double prob)
{
    if(prob==0)
        return;
    glPushMatrix();
    glTranslatef(pos.x(),pos.y(),0.0f);
    glScalef(1/12.0, 1/12.0, prob*3);
    //glRotatef(rotqube,0.0f,1.0f,0.0f);	// Rotate The cube around the Y axis
    //glRotatef(rotqube,1.0f,1.0f,1.0f);
    glBegin(GL_QUADS);		// Draw The Cube Using quads

    //glColor3f(1.0f,0.0f,1.0f);	// Color Violet
    glColor4f(0.0f,1.0f,0.0f,0.8f);

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
    glEnd();                            // End Drawing The Cube
    glPopMatrix();
}

void MapGL::renderSkeleton()
{
    if(!mapSkeleton)
        return;
    double shift=0;
    int colorIndex = 0;
    if(tasksGui->timeOfDay=="Morning")
    {
        glDeleteLists(morningList,1);
        glNewList(morningList,GL_COMPILE);
        shift = -0.30;
        colorIndex = 1;
    }
    else if(tasksGui->timeOfDay=="Early Morning")
    {
        glDeleteLists(earlyMorningList,1);
        glNewList(earlyMorningList,GL_COMPILE);
        shift = -0.10;
        colorIndex = 2;
    }
    else if(tasksGui->timeOfDay=="Afternoon")
    {
        glDeleteLists(afternoonList,1);
        glNewList(afternoonList,GL_COMPILE);
        shift = 0.10;
        colorIndex = 3;
    }
    else if(tasksGui->timeOfDay=="Night")
    {
        glDeleteLists(nightList,1);
        glNewList(nightList,GL_COMPILE);
        shift = 0.30;
        colorIndex = 4;
    }
    glPushMatrix();
    for(int i=0;i<mapSkeleton->getNumVerticies();i++)
    {
        QPointF parentVertex = mapSkeleton->verticies[i].getLocation();
        if(mapSkeleton->destIndexes.contains(i))
        {
            glBegin(GL_POLYGON);
                glColor4f(1,0,0,1);
                glVertex2f(parentVertex.x()- 0.2, parentVertex.y()+0.2);
                glVertex2f(parentVertex.x()+ 0.2, parentVertex.y()+0.2);
                glVertex2f(parentVertex.x()+ 0.2, parentVertex.y()-0.2);
                glVertex2f(parentVertex.x()- 0.2, parentVertex.y()-0.2);
            glEnd();
            glPushMatrix();
            QString str = QString("%1 \%").arg((int)(mapSkeleton->verticies[i].getProb()*100));
            glColor4f(0,0,0,1);
            QFont font; font.setPointSize(8);
            if(mapSkeleton->verticies[i].getProb()!=0)
                renderText(parentVertex.x(),parentVertex.y()+ 0.4, 0.2, str,font);
            glPopMatrix();
            drawProbHisto(parentVertex,mapSkeleton->verticies[i].getProb());
        }
        else
        {
            glBegin(GL_POLYGON);
                glColor4f(0.5,0.5,0.5,1);
                glVertex2f(parentVertex.x()- 0.2, parentVertex.y()+0.2);
                glVertex2f(parentVertex.x()+ 0.2, parentVertex.y()+0.2);
                glVertex2f(parentVertex.x()+ 0.2, parentVertex.y()-0.2);
                glVertex2f(parentVertex.x()- 0.2, parentVertex.y()-0.2);
            glEnd();
        }
        switch(colorIndex)
        {
        case 1:
            COLOR_YELLOW_A(0.8);
            break;
        case 2:
            COLOR_SIENNAL_A(0.8);
            break;
        case 3:
            COLOR_LIGHT_BLUE_A(0.8);
            break;
        case 4:
            COLOR_DEEP_PINK_A(0.8);
            break;
        default:
            COLOR_GREEN_A(0.8)
            //COLOR_YELLOW_A(0.8);
            //COLOR_LIGHT_BLUE_A(0.8);
            //COLOR_DEEP_PINK_A(0.8);
            //COLOR_SIENNAL_A(0.8);
        }
        for(int j=0;j<mapSkeleton->verticies[i].connections.size();j++)
        {
            QPointF child = mapSkeleton->verticies[mapSkeleton->verticies[i].connections[j].getNodeIndex()].getLocation();
            int k = mapSkeleton->verticies[i].connections[j].getNodeIndex();
            glLineWidth(mapSkeleton->getSegmentNumVisits(i,k)/10.0);
            glBegin(GL_LINES);
                glVertex2f(parentVertex.x()+shift,parentVertex.y()+shift);
                glVertex2f(child.x()+shift,child.y()+shift);
            glEnd();
        }
    }
    firstTime = false;
    glPopMatrix();
    if(colorIndex)
        glEndList();
}

void MapGL::renderPath()
{
    if(!tasksGui->voronoiPlanner)
    {
        return;
    }
    if(tasksGui->voronoiPlanner->path)
    {
        Node * path = tasksGui->voronoiPlanner->path;
        glPushMatrix();
        COLOR_BLUE_A(1)
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
    /* Enable Texture Mapping */
    glEnable(GL_TEXTURE_2D);
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
}

void MapGL::displayGrid()
{
    glPushMatrix();
    showGrids = true;
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
            //				glColor4f(1,1,1,0.5);
            glVertex3f(i-1,0.5,0);
            glVertex3f(i,0,0);
            glVertex3f(i-1,-0.5,0);
            glEnd();
            renderText(i,-1,0, "X");
        }
        //Y-axis indicator
        int j = int((ogMap->height*ogMap->mapRes)/2.0 + 2);
        {
            glBegin(GL_LINE_LOOP);
            glColor4f(0,0,0,0.5);
            //				glColor4f(1,1,1,0.5);
            glVertex3f(-0.5,j-1,0);
            glVertex3f(0,j,0);
            glVertex3f(0.5,j-1,0);
            glEnd();
            renderText(1,j,0, "Y");
        }
    }
    glPopMatrix();
}

/*!
 *  Renders The main Map loaded from the image file
 */
void MapGL::renderMap()
{
    glNewList(mapList, GL_COMPILE);
    /* Enable Texture Mapping */
    glEnable(GL_TEXTURE_2D);
    glPushMatrix();
    glBindTexture(GL_TEXTURE_2D, texId);
    // Inverse the Y-axis
    glScalef(1,-1,1);
    glTranslatef(-(newWidth*ogMap->mapRes)/2.0f,-(newHeight*ogMap->mapRes)/2.0f,0);
    //glColor4f(1,1,1,0.8);
    // Define Coordinate System
    glBegin(GL_QUADS);
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
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);

    glColor4f(0.0f,0.0f,0.0f,0.5f);
    renderText(0.65,-0.95, 0, (char*)"Scale:1m/Tile" );
    glColor4f(0.78f,0.78f,0.78f,0.5f);
    glRectf(0.64,-0.9f,aspectRatio-0.03,-0.96f);

    glPushMatrix();
    glScalef(1/zoomFactor, 1/zoomFactor, 1/zoomFactor);

    glRotatef(pitch,1,0,0);
    glRotatef(yaw,0,0,1);

    glTranslatef(xOffset, yOffset, zOffset);

    renderSkeleton();
    if(showPaths)
        renderPath();
    /*
    if(tasksGui->skeletonGenerated && firstTime)
    {
        glNewList(skeletonList, GL_COMPILE);
        glEndList();
        glCallList(skeletonList);
        firstTime = false;
    }
    else
        glCallList(skeletonList);
    */
    if(this->ogMap)
    {
        glDisable( GL_DEPTH_TEST );
        if(!mainMapBuilt)
        {
            loadTexture();
            renderMap();
        }
        displayGrid();
    }
    glCallList(mapList);
    glCallList(earlyMorningList);
    glCallList(morningList);
    glCallList(nightList);
    glCallList(afternoonList);
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
    loadTasks("./modules/TasksManager/tasks.txt");
}

void TasksGui::updateData()
{

}

void TasksGui::clearAll()
{
    LOG(Logger::Info,"Clearing GUI")
    playGround->mapManager->mapSkeleton.resetSegmentVisits();
    playGround->mapManager->mapSkeleton.resetVertexVisits();
    glDeleteLists(mapGL.morningList,1);
    glDeleteLists(mapGL.earlyMorningList,1);
    glDeleteLists(mapGL.nightList,1);
    glDeleteLists(mapGL.afternoonList,1);
    voronoiPlanner->freePath();
}

int TasksGui::config()
{
    QString commsName ="Wheelchair";
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
            unsigned int startVertex,endVertex,timeOfDay;
            string name;
            in >> startVertex >> endVertex >> timeOfDay >> name ;
            Task t(startVertex,endVertex,timeOfDay,name.c_str());
            tasks.push_back(t);

            QListWidgetItem *newItem = new QListWidgetItem;
            newItem->setText(name.c_str());
            tasksControlPanel.tasksList.insertItem(row++, newItem);
        }
    }
    else
    {
        LOG(Logger::Critical,"File Not Found")
    }
    for(int i=0; i<tasks.size();i++)
    {
        LOG(Logger::Info,QString("Task %1: from [%2 %3] to [%4 %5] Name:%6 ").arg(i+1).arg(tasks[i].getStart().x()).arg(tasks[i].getStart().y()) \
            .arg(tasks[i].getEnd().x()).arg(tasks[i].getEnd().y()).arg(qPrintable(tasks[i].getName())))
    }
}


void TasksGui::provideSpeed( double &in_speed, double &in_turnRate)
{
}

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

void TasksGui::saveImage()
{
    bool ok;
    QString filename = QInputDialog::getText(this, "Image Capture","Enter a name for the Image:", QLineEdit::Normal,
                                             QString::null, &ok);
    const char * type = "PNG";
    sleep(1);
    if(ok && !filename.isEmpty())
    {
        QImage capturedMap = mapGL.grabFrameBuffer();
        capturedMap.save(filename,type,-1);
    }
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

void TasksGui::timeOfDayChanged(QString _timeOfDay)
{
    timeOfDay = _timeOfDay;
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

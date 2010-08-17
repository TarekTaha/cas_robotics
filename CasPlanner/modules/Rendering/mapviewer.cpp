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
#include "mapviewer.h"
#include "bitmap_fonts.h"
#include "planningmanager.h"
#include "navigator.h"
#include "IntentionRecognizer.h"
#include "commmanager.h"
#include "socialplanner.h"

#define ROBOT 1

MapViewer::MapViewer(QWidget *parent,PlayGround *playG)
    : QGLWidget(QGLFormat(QGL::DoubleBuffer | QGL::Rgba | QGL::SampleBuffers), parent),
    step(1),
    playGround(playG),
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
    searchSpaceListCreated(false),
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
    connect(playGround,SIGNAL(mapUpdated(Map*)),this,       SLOT(updateMap(Map*)));

    RGB_COLOR[0][0] = 0.0; RGB_COLOR[0][1] = 0.7;   RGB_COLOR[0][2] = 0.7;   // Lightblue
    RGB_COLOR[1][0] = 1.0; RGB_COLOR[1][1] = 0.51;  RGB_COLOR[1][2] = 0.278; // Sienna1
    RGB_COLOR[2][0] = 0.0; RGB_COLOR[2][1] = 0.7;   RGB_COLOR[2][2] = 0.0;   // Green
    RGB_COLOR[3][0] = 0.7; RGB_COLOR[3][1] = 0.7;   RGB_COLOR[3][2] = 0.0;   // Yellow
    RGB_COLOR[4][0] = 0.8; RGB_COLOR[4][1] = 0.0;   RGB_COLOR[4][2] = 0.0;   // Red
    RGB_COLOR[5][0] = 1.0; RGB_COLOR[5][1] = 0.0;   RGB_COLOR[5][2] = 1.0;   // Magenta
    RGB_COLOR[6][0] = 0.0; RGB_COLOR[6][1] = 0.0;   RGB_COLOR[6][2] = 0.7;   // Blue
    RGB_COLOR[7][0] = 1.0; RGB_COLOR[7][1] = 0.65;  RGB_COLOR[7][2] = 0.0;   // Orange
    RGB_COLOR[8][0] = 1.0; RGB_COLOR[8][1] = 0.078; RGB_COLOR[8][2] = 0.576; // DeepPink
    RGB_COLOR[9][0] = 0.8; RGB_COLOR[9][1] = 0.0;   RGB_COLOR[9][2] = 0.0;   // Red

    showSearchSpaceSamples   = CasPlanner::settings().isShowSearchSpaceSamplesEnabled();
    showSearchSpaceTree      = CasPlanner::settings().isShowSearchSpaceTreeEnabled();
    showSearchTree           = CasPlanner::settings().isShowSearchTreeEnabled();
    showPath                 = CasPlanner::settings().isShowPathsEnabled();
    showRobotTrail           = CasPlanner::settings().isShowRobotTrailEnabled();
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
    glShadeModel(GL_SMOOTH);
    glClearDepth(1.0f);
    glEnable(GL_DEPTH_TEST);
    //    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

    beginRenderText(screenWidth,screenHeight);

    glFlush();
    mapList         = glGenLists(1);
    searchSpaceList = glGenLists(1);
    //    font2Render.init("resources/Test.ttf", 16);
    glInitNames();
}

void MapViewer::resizeGL(int w, int h)
{
    screenWidth = w;
    screenHeight = h;
    aspectRatio = ((float) w)/((float) h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(180, aspectRatio, 0.1,100);
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
    LOG(Logger::Info,"Updating Map")
    this->ogMap = newMap;
    mainMapBuilt = false;
    updateGL();
}

void MapViewer::renderPaths()
{
    if(!playGround)
    {
        LOG(Logger::Warning,"Can't render paths with empty Paths")
        exit(1);
    }
    for(int i=0;i<playGround->robotPlatforms.size();i++)
    {
        if(playGround->robotPlatforms[i]->planningManager->pathPlanner->path)
        {
            Node * path = playGround->robotPlatforms[i]->planningManager->pathPlanner->path;
            glColor4f(RGB_COLOR[6][0],RGB_COLOR[6][1],RGB_COLOR[6][2],0.5);
            if(showPath)
            {
                glLineWidth(2);
                glBegin(GL_LINE_STRIP);
                while(path)
                {
                    glVertex2f(path->pose.p.x(),path->pose.p.y());
                    path = path->next;
                }
                glEnd();
                glLineWidth(1);
            }
            //re-populate the path pointer
            path = playGround->robotPlatforms[i]->planningManager->pathPlanner->path;
            // draw the robot trail if wanted, this will show the location and
            // orientation of the robot at each step of the planned path
            if(showRobotTrail)
            {
                while(path)
                {
                    glPushMatrix();
                    glTranslatef(path->pose.p.x(),path->pose.p.y(),0);
                    glRotated(RTOD(path->pose.phi),0,0,1);
                    glColor4f(RGB_COLOR[3][0],RGB_COLOR[3][1],RGB_COLOR[3][2],0.5);
                    //glBegin(GL_TRIANGLE_FAN);
                    glBegin(GL_LINE_LOOP);
                    for(int m=0;m<playGround->robotPlatforms[i]->robot->local_edge_points.size();m++)
                    {
                        glVertex2f(playGround->robotPlatforms[i]->robot->local_edge_points[m].x(),playGround->robotPlatforms[i]->robot->local_edge_points[m].y());
                    }
                    glEnd();
                    glPopMatrix();
                    path = path->next;
                }
            }
            glLineWidth(1);
            wayPoint = playGround->robotPlatforms[i]->navigator->wayPoint;
            // Draw Way Point
            if(wayPoint.p.x()==0 && wayPoint.p.y()==0)
                continue;
            glPushMatrix();
                glTranslatef(wayPoint.p.x(),wayPoint.p.y(),0);
                glRotated(RTOD(wayPoint.phi),0,0,1);
                glColor4f(1,0,0,0.5);
                glShadeModel(GL_FLAT);
                glBegin(GL_TRIANGLE_FAN);
                glColor4f(1,0,0,0.5);
                    glVertex3f(0,0.1,0);
                    glVertex3f(0.3,0,0);
                    glVertex3f(0,-0.1,0);
                glEnd();
            glPopMatrix();
        }
        if(playGround->robotPlatforms[i]->intentionRecognizer)
        {
            if(playGround->robotPlatforms[i]->intentionRecognizer->socialPlanner)
            {
                glEnable(GL_LINE_SMOOTH);
                QHash<QString, int> socialRewards, socialTotalRewards;
                socialRewards = playGround->robotPlatforms[i]->intentionRecognizer->socialPlanner->getSocialRewards();
                socialTotalRewards = playGround->robotPlatforms[i]->intentionRecognizer->socialPlanner->getTotalSocialRewards();
                QHash<QString, int>::const_iterator j = socialRewards.constBegin();
                while (j != socialRewards.constEnd())
                {
                    int start,end,dest,destIndex;
                    sscanf(qPrintable(j.key()),"%d-%d-%d",&start,&end,&dest);
                    destIndex = playGround->mapManager->mapSkeleton.getDestIndex(dest-1);
                    //                        printf("\n Start:=%d End:=%d Dest:=%d",start,end,destIndex);
                    //                        printf("\n Edge:=%s Reward:=%d",qPrintable(j.key()),j.value());
                    double lineW=1.0;
                    if ((int)socialTotalRewards.value(QString("%1-%2").arg(start).arg(destIndex+1))!=0)
                    {
                        lineW = (double)j.value()/(double)socialTotalRewards.value(QString("%1-%2").arg(start).arg(dest));
                        if (lineW!=1.0)
                        {
                            printf("\n J-Value:%d SocialValue:=%d Width:=%f",j.value(),socialTotalRewards.value(QString("%1-%2").arg(start).arg(dest)),lineW);
                            printf("\n Start:=%d End:=%d Dest:=%d",start,end,destIndex);
                        }
                    }
                    double poseShift = -0.3 + 0.1*destIndex;
                    glColor4f(RGB_COLOR[destIndex%9][0],RGB_COLOR[destIndex%9][1],RGB_COLOR[destIndex%9][2],0.5f);
                    glLineWidth(1+lineW);
                    glBegin(GL_LINES);
                    glVertex2f(playGround->mapManager->mapSkeleton.verticies[start-1].getLocation().x() + poseShift,playGround->mapManager->mapSkeleton.verticies[start-1].getLocation().y() + poseShift);
                    glVertex2f(playGround->mapManager->mapSkeleton.verticies[end-1].getLocation().x() + poseShift,playGround->mapManager->mapSkeleton.verticies[end-1].getLocation().y() + poseShift);
                    glEnd();
                    ++j;
                }
                glDisable(GL_LINE_SMOOTH);
                glLineWidth(1);

                if(playGround->robotPlatforms[i]->intentionRecognizer->socialPlanner->getPath())
                {
                    Node * path = playGround->robotPlatforms[i]->intentionRecognizer->socialPlanner->getPath();
                    glColor4f(RGB_COLOR[3][0],RGB_COLOR[3][1],RGB_COLOR[3][2],0.5);
                    glLineWidth(2);
                    glColor4f(0,0,1,0.5);
                    glBegin(GL_LINE_STRIP);
                    while(path)
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
                    glColor4f(1,0,0,0.5);
                    glShadeModel(GL_FLAT);
                    glBegin(GL_TRIANGLE_FAN);
                    glColor4f(1,0,0,0.5);
                    glVertex3f(0,0.1,0);
                    glVertex3f(0.3,0,0);
                    glVertex3f(0,-0.1,0);
                    glEnd();
                    glPopMatrix();
                }
            }
        }
    }
}

void MapViewer::setRobotsLocation()
{
    robotsLocation.clear();
    for(int i=0;i<playGround->robotPlatforms.size();i++)
    {
        if(playGround->robotPlatforms[i]->commManager->isConnected())
            robotsLocation.push_back(playGround->robotPlatforms[i]->commManager->getLocation());
    }
}

void MapViewer::renderLaser()
{
    if(!playGround)
    {
        qDebug()<<"WTFFFF !!!";
        exit(1);
    }
    for(int i=0;i<playGround->robotPlatforms.size();i++)
    {
        if(!playGround->robotPlatforms[i]->commManager->isConnected())
            continue;
        LaserScan laserScan ;
        //laserScan =playGround->robotPlatforms[i]->commManager->getLaserScan();
        laserScan = playGround->robotPlatforms[i]->commManager->getLaserScan();
        //Pose loc = playGround->robotPlatforms[i]->robot->robotLocation;
        Pose loc;
        if(robotsLocation.size() > i )
            loc = robotsLocation[i];
        else
            continue;

        glPushMatrix();
        glTranslatef(loc.p.x(),loc.p.y(),0);
        glRotated(RTOD(loc.phi),0,0,1);
        glColor4f(140/double(255.0),231/double(255.0),237/double(255.0),0.5);
        glBegin(GL_POLYGON);
        glVertex2f(laserScan.laserPose.p.x(), laserScan.laserPose.p.y());
        if(laserScan.points.size() > 0)
        {
            for(int m=0; m < laserScan.points.size(); m++)
            {
                laserScan.points[m] = Trans2Global(laserScan.points[m],laserScan.laserPose);
                glVertex2f(laserScan.points[m].x(), laserScan.points[m].y());
            }
        }
        glVertex2f(laserScan.laserPose.p.x(), laserScan.laserPose.p.y());
        glEnd();

        glColor4f(41/double(255.0),107/double(255.0),112/double(255.0),0.5);
        glBegin(GL_LINE_LOOP);
        if(laserScan.points.size() > 0)
        {
            glVertex2f(laserScan.laserPose.p.x(), laserScan.laserPose.p.y());
            for(int m=0; m < laserScan.points.size(); m++)
                glVertex2f(laserScan.points[m].x(), laserScan.points[m].y());
        }
        glEnd();

        glTranslatef(laserScan.laserPose.p.x(), laserScan.laserPose.p.y(),0);
        glColor4f(255/double(255.0),50/double(255.0),50/double(255.0),0.8);
        glRectf(-0.025,0.025f, 0.025f, -0.025f);

        glPopMatrix();
    }
}

void MapViewer::renderSearchSpace()
{
    glNewList(searchSpaceList, GL_COMPILE);
    for(int i=0;i<1;i++)
    {
        SearchSpaceNode * temp,*child;
        if(!showSearchSpaceTree)
            continue;
        temp =  playGround->robotPlatforms[i]->planningManager->pathPlanner->search_space;
        /*
          If Intention Recognizer exists, then prefer the social planner
          over the path planner
          */
        if(playGround->robotPlatforms[i]->intentionRecognizer)
            if(playGround->robotPlatforms[i]->intentionRecognizer->socialPlanner)
                temp =  playGround->robotPlatforms[i]->intentionRecognizer->socialPlanner->getSearchSpace();
        glPushMatrix();
        glColor4f(1,0,0,0.5);
        while(temp)
        {
            glColor4f(1,0,0,0.5);
            glLineWidth(1);
            for(int j=0;j<temp->children.size();j++)
            {
                //qDebug("J is:%d",j); fflush(stdout);
                child = temp->children[j];
                if(!child)
                {
                    qDebug()<<"Why the hell there is an empty CHILD ???";
                    fflush(stdout);
                    continue;
                }
                glBegin(GL_LINE);
                    glVertex2f(temp->location.x(),temp->location.y());
                    glVertex2f(child->location.x(),child->location.y());
                glEnd();
            }
            if(showSearchSpaceSamples)
            {
                glColor4f(0,0,1,0.5);
                glBegin(GL_POINTS);
                    glVertex2f(temp->location.x(),temp->location.y());
                glEnd();
            }
            temp = temp->next;
        }
        glLineWidth(2);
        glPopMatrix();
    }
    glEndList();
    searchSpaceListCreated = true;
}

void MapViewer::renderExpandedTree()
{
    /* i should usually refer to the number of available robots
       but at this stage I am interested in only one.
       */
    for(int i=0;i<1;i++)
    {
        vector <Tree> tree;
        QPointF child;
        if(!showSearchTree)
            continue;
        if(playGround->robotPlatforms[i]->intentionRecognizer)
        {
            if(playGround->robotPlatforms[i]->intentionRecognizer->socialPlanner)
                tree =  playGround->robotPlatforms[i]->intentionRecognizer->socialPlanner->getTree();
            else
                continue;
        }
        else
        {
            if(playGround->robotPlatforms[i]->planningManager->pathPlanner)
                tree = playGround->robotPlatforms[i]->planningManager->pathPlanner->tree;
        }
        glPushMatrix();
        glColor4f(0,1,0,0.5);
        for(unsigned int k=0;k<tree.size();k++)
        {
            for(int j=0;j<tree[k].children.size();j++)
            {
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
        qDebug()<<"WHAT THEEEE !!!";
        exit(1);
    }
    for(int i=0;i<playGround->robotPlatforms.size();i++)
    {
        if(!playGround->robotPlatforms[i]->commManager->isConnected())
            continue;
        Pose loc;
        if(robotsLocation.size() > i )
            loc = robotsLocation[i];
        else
            continue;
        //	    qDebug("Robot Location is X:%f Y:%f Phi:%f",robotsLocation[i].p.x(),robotsLocation[i].p.y(),
        //	    	   robotsLocation[i].phi);
        // Render Robot's trail
        if(playGround->robotPlatforms[i]->robot->robotName=="Static Obstacle")
        {
            // Obstacle
            glPushMatrix();
            glPushName(ROBOT);
            glTranslated(loc.p.x(),loc.p.y(),0);
            glRotated(RTOD(loc.phi),0,0,1);
            glShadeModel(GL_FLAT);
            glColor4f(1,0,0,0.5);
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
            glColor4f(RGB_COLOR[i][0],RGB_COLOR[i][1],RGB_COLOR[i][2],0.5);
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
        //	    glShadeModel(GL_FLAT);
        // Robot Boundaries BOX 83b3c7
        //		glColor4f(RGB_COLOR[i][0],RGB_COLOR[i][1],RGB_COLOR[i][2],0.8);
        double colorRatio = 255.0;
        glColor4f(90/colorRatio,90/colorRatio,90/colorRatio,0.8);
        glBegin(GL_TRIANGLE_FAN);
        for(int m=0;m<playGround->robotPlatforms[i]->robot->local_edge_points.size();m++)
        {
            glVertex2f(playGround->robotPlatforms[i]->robot->local_edge_points[m].x(),playGround->robotPlatforms[i]->robot->local_edge_points[m].y());
        }
        glEnd();
        glColor4f(0,0,1,0.5);
        QFont font40; font40.setPointSize(10);
        //	    renderText(1.6,0,1, qPrintable(playGround->robotPlatforms[i]->robot->robotName),font40);
        renderTextFont(1.6,0, BITMAP_FONT_TYPE_HELVETICA_12, (char *)qPrintable(playGround->robotPlatforms[i]->robot->robotName));
        glColor4f(0,0,1,0.5);
        //	    freetype::print(font2Render,1.6,0, "Active FreeType Text");

        glBegin(GL_TRIANGLE_FAN);
        glColor4f(0,0,1,0.5);
        glVertex3f(1.3, 0.15,0);
        glVertex3f(1.5,0,0);
        glVertex3f(1.3,-0.15,0);
        glVertex3f(1.3, 0.15,0);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glColor4f(0,0,1,0.5);
        glVertex3f(0,0,0);
        glVertex3f(1.5,0,0);
        glEnd();
        // This will display the circles covering the robot for Obstacle Avoidance reasons
        //		glColor4f(0,1,0,1);
        //		for (int k=0;k<playGround->robotPlatforms[i]->robot->check_points.size();k++)
        //		{
        //			drawCircle(playGround->robotPlatforms[i]->robot->check_points[k],playGround->robotPlatforms[i]->robot->expansionRadius);
        //		}

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
                imgData[(j*ogMap->width+i)*4+3] = 50;
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
    //      	if (gluScaleImage(GL_RGB_COLORA, ogMap->width, ogMap->height,
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
            glColor4f(0.0f,0.0f,0.0f,1.0f);
            renderText(i,-1,1, "X");
            //renderTextFont(i,-1, BITMAP_FONT_TYPE_HELVETICA_18, (char*)"X-axis");
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
            glColor4f(0.0f,0.0f,0.0f,1.0f);
            renderText(1,j,1, "Y");
            //renderTextFont(1,j, BITMAP_FONT_TYPE_HELVETICA_18, (char*)"Y-axis");
        }
    }
    glPopMatrix();
}

void MapViewer::drawCircle(float radius)
{
    glBegin(GL_LINE_STRIP);
    for (int i=0; i < 360; i++)
    {
        float degInRad = DTOR(i);
        glVertex2f(cos(degInRad)*radius,sin(degInRad)*radius);
    }
    glEnd();
}

void MapViewer::drawCircle(QPointF center,float radius)
{
    glBegin(GL_LINE_STRIP);
    for (int i=0; i < 360; i++)
    {
        float degInRad = DTOR(i);
        glVertex2f(cos(degInRad)*radius+center.x(),sin(degInRad)*radius+center.y());
    }
    glEnd();
}

void MapViewer::renderObservation()
{
    playGround->activeRobot->commManager->getJoyStickGlobalDir();
    if(!playGround->mapManager ||!playGround->activeRobot->commManager)//|| !playGround->activeRobot->intentionRecognizer)
        return;

    glPushMatrix();
    glLineWidth(1);
    glColor4f(0.0f,0.0f,1.0f,1.0f);
    glTranslatef(-aspectRatio+0.1,0.9,0.0);
    glScalef(1/15.0, 1/15.0, 1.0);
    renderTextFont(-0.99,0.95, BITMAP_FONT_TYPE_HELVETICA_12,(char*)"Joystick");
    drawCircle(1.0);
    //int obs = playGround->activeRobot->intentionRecognizer->lastObs;
    int obs = playGround->activeRobot->commManager->getJoyStickGlobalDir();
    //int obs = playGround->activeRobot->commManager->getJoyStickDir();
    glLineWidth(2);
    switch (obs)
    {
                        case 0:
        glLineWidth(2);
        glBegin(GL_LINES);
        glVertex2f( 0.0, 0.0);
        glVertex2f( 0.0, 1.0);
        glEnd();
        glBegin(GL_TRIANGLE_FAN);
        glVertex2f(-0.2, 0.7);
        glVertex2f( 0.0, 1.0);
        glVertex2f( 0.2, 0.7);
        glVertex2f(-0.2, 0.7);
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
    if(!playGround->mapManager || !playGround->activeRobot->intentionRecognizer||!playGround->activeRobot->commManager)
        return;
}

void MapViewer::renderSpatialStates()
{
    if(!playGround->mapManager || !playGround->activeRobot->intentionRecognizer||!playGround->activeRobot->commManager)
        return;
    // This is not the general Case now and i might need to change it
    Pose l = playGround->activeRobot->commManager->getLocation();
    for(int i=0; i < playGround->mapManager->mapSkeleton.verticies.size() ;i++)
    {
        int d;
        /*
                glLineWidth(2);
            for(int k =0; k<playGround->mapManager->mapSkeleton.verticies[i].connections.size();k++)
            {
                QPointF s = playGround->mapManager->mapSkeleton.verticies[i].location;
                int index = playGround->mapManager->mapSkeleton.verticies[i].connections[k].nodeIndex;
                QPointF e = playGround->mapManager->mapSkeleton.verticies[index].location;
                        glBegin(GL_LINES);
                                glVertex2f(s.x(),s.y());
                                glVertex2f(e.x(),e.y());
                        glEnd();
            }
            glLineWidth(1);
            */
        glPushMatrix();
        glTranslated(playGround->mapManager->mapSkeleton.verticies[i].getLocation().x(),playGround->mapManager->mapSkeleton.verticies[i].getLocation().y(),0);
        glShadeModel(GL_FLAT);
        if(i==playGround->mapManager->mapSkeleton.getCurrentSpatialState(l))
        {
            glColor4f(1,0,0,0.5);
            //	    	renderTextFont(0,0, BITMAP_FONT_TYPE_HELVETICA_10,"Current");
        }
        else if (i == playGround->activeRobot->intentionRecognizer->nextState)
        {
            glColor4f(0,0,1,0.5);
            renderTextFont(-0.5,0.5, BITMAP_FONT_TYPE_HELVETICA_10,(char*)"Next");
        }
        else if( (d = playGround->mapManager->mapSkeleton.destIndexes.indexOf((i%playGround->mapManager->mapSkeleton.numStates)))!=-1 )
        {
            glColor4f(RGB_COLOR[d%9][0],RGB_COLOR[d%9][1],RGB_COLOR[d%9][2],0.5f);
            renderTextFont(-0.1,0.3, BITMAP_FONT_TYPE_HELVETICA_12,(char*)qPrintable(QString("%1").arg(d+1)));
        }
        else
            glColor4f(0.33,0.33,0.33,0.5);
        glRectf(-0.2f,0.2f, 0.2f, -0.2f);
        glPopMatrix();
    }

}

void MapViewer::renderDestIndicators()
{
    if(!playGround->mapManager || !playGround->activeRobot->intentionRecognizer ||!playGround->activeRobot->commManager)
        return;
    glColor4f(1.0f,0.0f,0.0f,1.0f);
    //renderTextFont(-0.83,0.87, BITMAP_FONT_TYPE_HELVETICA_18,"Probs:");
    for(int i=0; i < playGround->activeRobot->intentionRecognizer->numDestinations ;i++)
    {
        int r;
        r = playGround->mapManager->mapSkeleton.destIndexes[i];
        //		drawProbHisto(playGround->mapManager->mapSkeleton.verticies[r].location, playGround->activeRobot->intentionRecognizer->destBelief[i]);
        glPushMatrix();
        glColor4f(RGB_COLOR[i%9][0],RGB_COLOR[i%9][1],RGB_COLOR[i%9][2],1.0f);
        renderTextFont(-0.9 + floor(i/3.0)*0.25,0.91 - (i%3)*0.035, BITMAP_FONT_TYPE_HELVETICA_12,(char*)qPrintable(QString("Dest %1:").arg(i+1)));
        glTranslatef(-0.75  + floor(i/3.0)*0.25,0.91 - (i%3)*0.035 ,0.0);
        glScalef(playGround->activeRobot->intentionRecognizer->destBelief[i]*4.0f,1.0f, 1.0f);
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
    glColor4f(1.0f,0.5f,0.0f,0.5f);
    // 	renderText(0 ,0 + 0.2, prob+ 0.2, str);
    //	glScalef(1/12.0, 1/12.0, prob);
    //glRotatef(rotqube,0.0f,1.0f,0.0f);	// Rotate The cube around the Y axis
    //glRotatef(rotqube,1.0f,1.0f,1.0f);
    glBegin(GL_QUADS);		// Draw The Cube Using quads

    glColor4f(0.0f,1.0f,0.0f,0.5f);

    glVertex3f( 1.0f, 1.0f,-0.0f);	// Top Right Of The Quad (Top)
    glVertex3f(-1.0f, 1.0f,-0.0f);	// Top Left Of The Quad (Top)
    glVertex3f(-1.0f, 1.0f, 1.0f);	// Bottom Left Of The Quad (Top)
    glVertex3f( 1.0f, 1.0f, 1.0f);	// Bottom Right Of The Quad (Top)

    glVertex3f( 1.0f,-1.0f, 1.0f);	// Top Right Of The Quad (Bottom)
    glVertex3f(-1.0f,-1.0f, 1.0f);	// Top Left Of The Quad (Bottom)
    glVertex3f(-1.0f,-1.0f,-0.0f);	// Bottom Left Of The Quad (Bottom)
    glVertex3f( 1.0f,-1.0f,-0.0f);	// Bottom Right Of The Quad (Bottom)

    glVertex3f( 1.0f, 1.0f, 1.0f);	// Top Right Of The Quad (Front)
    glVertex3f(-1.0f, 1.0f, 1.0f);	// Top Left Of The Quad (Front)
    glVertex3f(-1.0f,-1.0f, 1.0f);	// Bottom Left Of The Quad (Front)
    glVertex3f( 1.0f,-1.0f, 1.0f);	// Bottom Right Of The Quad (Front)

    glVertex3f( 1.0f,-1.0f,-0.0f);	// Top Right Of The Quad (Back)
    glVertex3f(-1.0f,-1.0f,-0.0f);	// Top Left Of The Quad (Back)
    glVertex3f(-1.0f, 1.0f,-0.0f);	// Bottom Left Of The Quad (Back)
    glVertex3f( 1.0f, 1.0f,-0.0f);	// Bottom Right Of The Quad (Back)

    glVertex3f(-1.0f, 1.0f, 1.0f);	// Top Right Of The Quad (Left)
    glVertex3f(-1.0f, 1.0f,-0.0f);	// Top Left Of The Quad (Left)
    glVertex3f(-1.0f,-1.0f,-0.0f);	// Bottom Left Of The Quad (Left)
    glVertex3f(-1.0f,-1.0f, 1.0f);	// Bottom Right Of The Quad (Left)

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
    if(start_initialized)
    {
        if(step == 2)
        {
            double phi = atan2((mousePos.y()-start.p.y()),(mousePos.x()-start.p.x()));
            // Rotation Line
            glPushMatrix();
            glBegin(GL_LINE_LOOP);
            COLOR_GREEN_A(1)
            glVertex3f(start.p.x(),start.p.y(),0);
            glVertex3f(mousePos.x(),mousePos.y(),0);
            glEnd();
            glPopMatrix();
            // Rotate the triangle based on the arrow
            glPushMatrix();
            glTranslatef(start.p.x(),start.p.y(),0);
            glRotated(RTOD(phi),0,0,1);
            glShadeModel(GL_FLAT);
            // Path Start
            glBegin(GL_TRIANGLE_FAN);
            COLOR_DARK_RED_A(0.8)
            glVertex3f(-0.2,0.15,0);
            glVertex3f(0.3,0,0);
            glVertex3f(-0.2,-0.15,0);
            glEnd();
            glPopMatrix();
        }
        else
        {
            glPushMatrix();
            glTranslatef(start.p.x(),start.p.y(),0);
            glRotated(RTOD(start.phi),0,0,1);
            glColor4f(1,1,1,0.5);
            glShadeModel(GL_FLAT);
            // Path Start
            glBegin(GL_TRIANGLE_FAN);
            COLOR_DARK_RED_A(0.8)
            glVertex3f(-0.2,0.15,0);
            glVertex3f(0.3,0,0);
            glVertex3f(-0.2,-0.15,0);
            glEnd();
            //Text
            glColor4f(0,0,0,0.8);
            QFont font40; font40.setPointSize(6);
            renderText(0.2,0.2,0,QString("Start"), font40);
            glPopMatrix();
        }
    }
    if(end_initialized)
    {
        if(step == 4)
        {
            double phi = atan2((mousePos.y()-end.p.y()),(mousePos.x()-end.p.x()));
            // Rotation Line
            glPushMatrix();
            glBegin(GL_LINE_LOOP);
            COLOR_GREEN_A(1)
            glVertex3f(end.p.x(),end.p.y(),0);
            glVertex3f(mousePos.x(),mousePos.y(),0);
            glEnd();
            glPopMatrix();
            // Rotate the triangle based on the arrow
            glPushMatrix();
            glTranslatef(end.p.x(),end.p.y(),0);
            glRotated(RTOD(phi),0,0,1);
            glShadeModel(GL_FLAT);
            // Path Start
            glBegin(GL_TRIANGLE_FAN);
            COLOR_SIENNAL_A(0.8)
            glVertex3f(-0.2,0.15,0);
            glVertex3f(0.3,0,0);
            glVertex3f(-0.2,-0.15,0);
            glEnd();
            glPopMatrix();
        }
        else
        {
            glPushMatrix();
            glTranslatef(end.p.x(),end.p.y(),0);
            glRotated(RTOD(end.phi),0,0,1);
            glColor4f(1,1,1,0.5);
            glShadeModel(GL_FLAT);
            // Path Start
            glBegin(GL_TRIANGLE_FAN);
            COLOR_SIENNAL_A(0.8)
            glVertex3f(-0.2,0.15,0);
            glVertex3f(0.3,0,0);
            glVertex3f(-0.2,-0.15,0);
            glEnd();
            //Draw the allowed Range Circle
            COLOR_GREEN_A(0.8)
            drawCircle(QPointF(0,0),playGround->activeRobot->planningManager->pathPlanner->distGoal);
            //Text
            glColor4f(0,0,0,0.8);
            QFont font40; font40.setPointSize(6);
            renderText(0.2,0.2,0,QString("End"), font40);
            glPopMatrix();
        }
    }
}

void MapViewer::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_ALPHA,GL_ONE);
    //    glDisable(GL_BLEND);
    //    glBlendFunc(GL_ONE, GL_ONE);
    //    glBlendFunc(GL_ONE_MINUS_SRC_COLOR, GL_ONE_MINUS_SRC_ALPHA);
    //    glBlendFunc(GL_SRC_COLOR, GL_SRC_ALPHA);
    //    glBlendFunc(GL_DST_COLOR, GL_DST_ALPHA);
    //    glBlendFunc(GL_DST_COLOR, GL_ZERO);


    glEnable(GL_DEPTH_TEST);
    //    glDisable( GL_DEPTH_TEST );
    //    glDisable( GL_LIGHTING );
    //    glDisable(GL_DEPTH_TEST);
    //    glEnable(GL_POINT_SMOOTH);
    //    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    //    glEnable(GL_LINE_SMOOTH);
    //    glEnable(GL_POLYGON_SMOOTH);
    glColor4f(0.0f,0.0f,0.0f,0.5f);
    renderTextFont(0.65,-0.95, BITMAP_FONT_TYPE_HELVETICA_18, (char*)"Scale:1m/Tile" );
    glColor4f(0.78f,0.78f,0.78f,0.8f);
    glRectf(0.64,-0.9f,aspectRatio-0.03,-0.96f);

    renderObservation();
    renderDestIndicators();

    glColor4f(0.78f,0.78f,0.78f,0.5f);
    glRectf(-aspectRatio+0.01,0.8f,-0.4f, 1.0f-0.01);

    glColor4f(0.0f,0.0f,0.0f,0.5f);
    glLineWidth(3);
    glBegin(GL_LINE_STRIP);
    glVertex3f(-aspectRatio+0.01,0.8,0);
    glVertex3f(-aspectRatio+0.01,1.0-0.01,0);
    glVertex3f(-0.4,1.0-0.01,0);
    glVertex3f(-0.4,0.8,0);
    glVertex3f(-aspectRatio+0.01,0.8,0);
    glEnd();
    glLineWidth(1);
    glPushMatrix();
    glScalef(1/zoomFactor, 1/zoomFactor, 1/zoomFactor);
    //    renderText(zoomFactor*aspectRatio*0.90 - 2, -0.9*zoomFactor, 1, "grid: 1 m");

    glRotatef(pitch,1,0,0);
    glRotatef(yaw,0,0,1);
    glTranslatef(xOffset, yOffset, zOffset);

    glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
    glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
    glGetIntegerv(GL_VIEWPORT,viewport);
    setRobotsLocation();
    if(this->ogMap)
    {
        glDisable( GL_DEPTH_TEST );
        if(!mainMapBuilt)
        {
            loadTexture();
            renderMap();
        }
        displayGrid();
        showIndicators();
        renderPaths();
        renderLaser();
        renderRobot();
        //glEnable(GL_DEPTH_TEST);
        renderSpatialStates();
        renderAction();
        renderExpandedTree();
        if(!searchSpaceListCreated)
            renderSearchSpace();
        glCallList(searchSpaceList);
    }
    glCallList(mapList);
    glPopMatrix();
}

void MapViewer::searchSpaceGenerated()
{
    searchSpaceListCreated = false;
    LOG(Logger::Info,"SearchSpace Generated- SIGNAL Emitted")
}

void MapViewer::setShowSearchSpaceSamples(bool state)
{
    glDeleteLists(searchSpaceList,1);
    searchSpaceListCreated = false;
    showSearchSpaceSamples = state;
}

void MapViewer::setShowSearchSpaceTree(bool state)
{
    glDeleteLists(searchSpaceList,1);
    searchSpaceListCreated = false;
    showSearchSpaceTree = state;
}

void MapViewer::setShowSearchTree(bool state)
{
    showSearchTree = state;
}

void MapViewer::setShowPath(bool state)
{
    showPath = state;
}

void MapViewer::setShowRobotTrail(bool state)
{
    showRobotTrail = state;
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

void MapViewer::wheelEvent( QWheelEvent * event )
{
    int numDegrees = event->delta() / 8;
    int numSteps = numDegrees / 15;
    if(numSteps > 0)
        zoomFactor /= 1.1;
    else
        zoomFactor *= 1.1;

}

void MapViewer::mouseDoubleClickEvent(QMouseEvent *me)
{
    //	updateMap(playGround->activeRobot->planningManager->pathPlanner->map);
    QPointF p(me->x(),me->y());
    //qDebug("Mouse Double click x: %f y: %f",p.x(),p.y());
    p = getOGLPos(p.x(),p.y());
    if(me->buttons()==Qt::RightButton)
    {
        newLocation.p = p;
    }
    else if(step == 1)
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
    if(me->buttons()==Qt::RightButton)
    {
        newLocation.phi = atan2(p.y() - newLocation.p.y(),p.x() - newLocation.p.x());
        playGround->activeRobot->commManager->setLocation(newLocation);
    }
    else if(step ==2)
    {
        start.phi = atan2(p.y() - start.p.y(),p.x() - start.p.x());
        Q_EMIT setStart(start);
        LOG(Logger::Info,"Start Angle ="<<RTOD(start.phi))
        step++;
        update();
        setMouseTracking(false);
    }
    else if(step == 4)
    {
        end.phi = atan2(p.y() - end.p.y(),p.x() - end.p.x());
        LOG(Logger::Info,"End Angle ="<<RTOD(end.phi))
        Q_EMIT setEnd(end)	;
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
            Q_EMIT moveMOUp();
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
            Q_EMIT moveMODown();
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
            Q_EMIT moveMOLeft();
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
            Q_EMIT moveMORight();
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
            Q_EMIT yawMOPos();
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
            Q_EMIT yawMONeg();
        }
        else
        {
            yaw -= 5;
        }
    }
    else if(e->key() == Qt::Key_Up)
    {
        pitch += 5;
    }
    else if(e->key() == Qt::Key_Down)
    {
        pitch -= 5;
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

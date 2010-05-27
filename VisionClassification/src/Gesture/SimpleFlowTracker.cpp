/***************************************************************************
 *   Vision Classification Library                                         *
 *   Copyright (C) 2010 by:                                                *
 *      Tarek Taha, CAS-UTS  <tataha@cas.edu.au>                           *
 *      Dan Maynes-Aminzade  <monzy@stanford.edu>                          *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02110-1301, USA.          *
 ***************************************************************************/
#include "SimpleFlowTracker.h"

SimpleFlowTracker::SimpleFlowTracker():
        graphics(NULL)
{
    isInitialized = false;
}

SimpleFlowTracker::SimpleFlowTracker(Graphics *_graphics):
        graphics(_graphics)
{
    isInitialized = false;
}


SimpleFlowTracker::~SimpleFlowTracker()
{
    if (isInitialized) StopTracking();
}

void SimpleFlowTracker::StartTracking(IplImage *firstFrame)
{
    // Allocate images to store previous frame
    outputFrame = cvCloneImage(firstFrame);
    int width = firstFrame->width;
    int height = firstFrame->height;

    // Allocate images to store grayscale copies
    grcurrentFrame = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
    cvConvertImage(firstFrame, grcurrentFrame);
    grlastFrame = cvCloneImage(grcurrentFrame);

    // Allocate some arrays needed as workspace for the optical flow algorithm
    eigimage = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
    tempimage = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
    pyramid1 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
    pyramid2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);

    currentX = width/2;
    currentY = height/2;

    isInitialized = true;
    numInactiveFrames = 0;
}

void SimpleFlowTracker::StopTracking()
{
    if (!isInitialized)
        return;

    cvReleaseImage(&outputFrame);
    cvReleaseImage(&grcurrentFrame);
    cvReleaseImage(&grlastFrame);
    cvReleaseImage(&eigimage);
    cvReleaseImage(&tempimage);
    cvReleaseImage(&pyramid1);
    cvReleaseImage(&pyramid2);

    isInitialized = false;
}


void SimpleFlowTracker::ProcessFrame(IplImage *frame)
{
    if (!isInitialized)
    {
        StartTracking(frame);
    }

    cvCopy(frame, outputFrame);
    cvCopy(grcurrentFrame, grlastFrame);
    cvConvertImage(frame, grcurrentFrame);

    // Pick out the good features in the image
    int nFeatures = FLOW_MAX_TRACK_FEATURES;
    cvGoodFeaturesToTrack(grlastFrame, eigimage, tempimage, lastframe_features, &nFeatures, 0.01, 10);

    double mvecx = 0, mvecy = 0, magnitude = 0, npoints = 0;
    if (nFeatures > 0)
    {
        cvFindCornerSubPix(grlastFrame, lastframe_features, nFeatures, cvSize(5,5), cvSize(-1,-1),
                           cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));

        // Run the optical flow algorithm
        cvCalcOpticalFlowPyrLK(grlastFrame, grcurrentFrame, pyramid1, pyramid2,
                               lastframe_features, currframe_features, nFeatures,
                               cvSize(5,5), 3, found_features, feature_error,
                               cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ), 0);

        cvFindCornerSubPix(grcurrentFrame, currframe_features, nFeatures, cvSize(5,5), cvSize(-1,-1),
                           cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));

        // Now there is a 1 in found_features wherever two features match up
        // The feature positions are in lastframe_features and frame_features
        // Average all the flow vectors to get the direction
        for (int i=0; i<nFeatures; i++)
        {
            if ((found_features[i] == 0) || (feature_error[i]>FLOW_MAX_ERROR_THRESHOLD)) continue;
            CvPoint p,q;
            p.x = lastframe_features[i].x;
            p.y = lastframe_features[i].y;
            q.x = currframe_features[i].x;
            q.y = currframe_features[i].y;
            mvecx += (currframe_features[i].x-lastframe_features[i].x);
            mvecy += (currframe_features[i].y-lastframe_features[i].y);
            npoints++;
        }
        if (npoints>0)
        {
            mvecx /= npoints;
            mvecy /= npoints;
        }
        magnitude = hypot(mvecx, mvecy);
        if (magnitude < FLOW_MIN_MOTION_THRESHOLD)
        {
            mvecx = 0;
            mvecy = 0;
            numInactiveFrames++;
            if (numInactiveFrames > FLOW_INACTIVE_THRESHOLD)
            {
                ClearCurrentTrajectory();
                numInactiveFrames = 0;
            }
        }
        else
        {
            numInactiveFrames = 0;
        }
    }
    else
    {	// no trackable features were detected (similar to case when motion level is below threshold)
        numInactiveFrames++;
        if (numInactiveFrames > FLOW_INACTIVE_THRESHOLD)
        {
            ClearCurrentTrajectory();
            numInactiveFrames = 0;
        }
    }
    currentX += mvecx;
    currentY += mvecy;
    OneDollarPoint pt(currentX, currentY);
    trajectory.push_back(pt);

    // copy the points into a vector and rescale them so we can draw the trajectory in the image
    MotionTrack scaledpts;
    for (list<OneDollarPoint>::iterator i = trajectory.begin(); i != trajectory.end(); i++)
    {
        scaledpts.push_back(*i);
    }
    scaledpts = ScaleToSquare(scaledpts, 2*GESTURE_SQUARE_SIZE/3);
    scaledpts = TranslateToOrigin(scaledpts);
    if(graphics)
        graphics->drawTrack(outputFrame, scaledpts, CV_RGB(100,255,100), 3, GESTURE_SQUARE_SIZE,scaledpts.size());

    // don't let the current trajectory grow longer than the max allowed trajectory length
    if (trajectory.size() > GESTURE_MAX_TRAJECTORY_LENGTH)
        trajectory.pop_front();
}

MotionTrack SimpleFlowTracker::GetCurrentTrajectory()
{
    MotionTrack mt;
    for (list<OneDollarPoint>::iterator i = trajectory.begin(); i != trajectory.end(); i++)
    {
        mt.push_back(*i);
    }
    return mt;
}

void SimpleFlowTracker::ClearCurrentTrajectory()
{
    // this function is called after a successful match when running live
    trajectory.clear();
}

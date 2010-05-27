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
#include "VideoLoader.h"

CVideoLoader::CVideoLoader()
{
    videoLoaded = FALSE;
    videoCapture = NULL;
    copyFrame = NULL;
    motionHistory = NULL;
    nFrames = 0;
    currentFrameNumber = 0;
    m_flowTracker = NULL;
    guessMask = NULL;
    maskedFrame = NULL;
}

CVideoLoader::~CVideoLoader(void)
{
    if (videoLoaded)
    {
        cvReleaseCapture(&videoCapture);
        cvReleaseImage(&copyFrame);
        cvReleaseImage(&motionHistory);
        cvReleaseImage(&guessMask);
        cvReleaseImage(&maskedFrame);
        if (m_flowTracker != NULL)
            delete m_flowTracker;
    }
}

bool CVideoLoader::openVideoFile(const char * filename)
{
    char szFileName[MAX_PATH] = "";
    strcpy(szFileName, filename);

    // Attempt to load the video file and get dimensions
    CvCapture *vc = cvCreateFileCapture( (szFileName));

    if (vc == NULL)
    {
        return FALSE;
    }

    if (videoLoaded)  // We already loaded a video, so this will be a new one
    {
        cvReleaseCapture(&videoCapture);
        cvReleaseImage(&copyFrame);
        cvReleaseImage(&motionHistory);
        videoLoaded = false;
    }
    videoCapture = vc;

    currentFrame = cvQueryFrame(videoCapture);
    videoX = cvGetCaptureProperty(videoCapture, CV_CAP_PROP_FRAME_WIDTH);
    videoY = cvGetCaptureProperty(videoCapture, CV_CAP_PROP_FRAME_HEIGHT);
    nFrames = cvGetCaptureProperty(videoCapture,  CV_CAP_PROP_FRAME_COUNT);

    // create an image to store a copy of the current frame
    copyFrame = cvCreateImage(cvSize(videoX,videoY),IPL_DEPTH_8U,3);

    // create an image to store the motion history (for motion tracking)
    motionHistory = cvCreateImage(cvSize(videoX,videoY), IPL_DEPTH_32F, 1);

    // Create images to store masked version of frame
    guessMask = cvCreateImage(cvSize(videoX,videoY),IPL_DEPTH_8U,1);
    maskedFrame = cvCreateImage(cvSize(videoX,videoY),IPL_DEPTH_8U,3);
    cvSet(guessMask, cvScalar(0xFF));

    if (nFrames == 0)
    {
        // this is not a seekable format, so we will convert it and save to a file
        strcat(szFileName,".seekable.avi");
        videoWriter = cvCreateVideoWriter( (szFileName), CV_FOURCC('D','I','V','X'), 30, cvSize(videoX, videoY), 1);
        cvReleaseVideoWriter(&videoWriter);
        cvReleaseCapture(&videoCapture);
        videoCapture = cvCreateFileCapture( (szFileName));
    }

    loadFrame(0);
    videoLoaded = TRUE;

    // create a new blob tracker for the new video
    if (m_flowTracker != NULL)
        delete m_flowTracker;
    m_flowTracker = new FlowTracker();
    return TRUE;
}

void CVideoLoader::loadFrame(long framenum)
{
    if (!videoLoaded)
        return;
    cvSetCaptureProperty(videoCapture, CV_CAP_PROP_POS_FRAMES, framenum);
    currentFrame = cvQueryFrame(videoCapture);
    if (!currentFrame)
        return;
    if ((currentFrame->depth != IPL_DEPTH_8U) || (currentFrame->nChannels != 3))
        return;

    if (currentFrame->origin  == IPL_ORIGIN_TL)
    {
        cvCopy(currentFrame,copyFrame);
    }
    else
    {
        cvFlip(currentFrame,copyFrame);
    }

    currentFrameNumber = framenum;
}

IplImage* CVideoLoader::getMaskedBitmap()
{
    // create masked version of current frame
    cvZero(maskedFrame);
    cvCopy(copyFrame, maskedFrame, guessMask);
    cvAddWeighted(copyFrame, 0.15, maskedFrame, 0.85, 0.0, maskedFrame);

    // draw mask outlines in green using cvFindContours
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contours = NULL;
    cvFindContours(guessMask, storage, &contours, sizeof(CvContour),
                   CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );
    cvDrawContours(maskedFrame, contours, CV_RGB(100,255,100), CV_RGB(0,0,0), 1, 1, 8);
    return maskedFrame;
}

void CVideoLoader::convertFrame()
{
    if (currentFrame == NULL) return;

    CvRect videoBounds = cvRect(0,0,videoX,videoY);
    if (currentFrame->origin  == IPL_ORIGIN_TL)
    {
        cvCopy(currentFrame,copyFrame);
    }
    else
    {
        cvFlip(currentFrame,copyFrame);
    }
    cvWriteFrame(videoWriter, copyFrame);
    nFrames++;
    // Grab next frame
    currentFrame = cvQueryFrame(videoCapture);
}

IplImage* CVideoLoader::getMotionHistory()
{
    if (!videoLoaded)
        return NULL;
    cvZero(motionHistory);

    if (currentFrameNumber < MOTION_NUM_IMAGES+1)
        return motionHistory;

    // Allocate an image history ring buffer
    IplImage* buf[MOTION_NUM_IMAGES];
    memset(buf, 0, MOTION_NUM_IMAGES*sizeof(IplImage*));
    for(int i = 0; i < MOTION_NUM_IMAGES; i++)
    {
        buf[i] = cvCreateImage(cvSize(copyFrame->width,copyFrame->height), IPL_DEPTH_8U, 1);
        cvZero(buf[i]);
    }

    int last = 0;
    int idx1 = 0;
    int idx2 = 0;
    IplImage* silh = NULL;

    int startFrame = max(0, currentFrameNumber-MOTION_NUM_HISTORY_FRAMES);

    for (int i=startFrame; i<=currentFrameNumber; i++)
    {

        cvSetCaptureProperty(videoCapture, CV_CAP_PROP_POS_FRAMES, i);
        currentFrame = cvQueryFrame(videoCapture);
        if (!currentFrame) return motionHistory;

        if (currentFrame->origin  == IPL_ORIGIN_TL) {
            cvCopy(currentFrame,copyFrame);
        } else {
            cvFlip(currentFrame,copyFrame);
        }
        
        // convert frame to grayscale
        cvCvtColor(copyFrame, buf[last], CV_BGR2GRAY);
        idx1 = last;
        idx2 = (last + 1) % MOTION_NUM_IMAGES;
        last = idx2;

        silh = buf[idx2];
        cvAbsDiff( buf[idx1], buf[idx2], silh ); // get difference between frames
        
        cvThreshold(silh, silh, MOTION_DIFF_THRESHOLD, 1, CV_THRESH_BINARY ); // and threshold it
        if (i != startFrame)
        {
            cvUpdateMotionHistory(silh, motionHistory, (i-startFrame), MOTION_MHI_DURATION); // update MHI
        }
    }
    for(int i = 0; i < MOTION_NUM_IMAGES; i++)
    {
        cvReleaseImage(&buf[i]);
    }
    return motionHistory;
}

void CVideoLoader::learnTrajectories()
{
    if (!videoLoaded)
        return;
    if (!m_flowTracker)
        return;
    if (m_flowTracker->isTrained)
        return;
    m_flowTracker->learnTrajectories(videoCapture);
    loadFrame(currentFrameNumber);
}

MotionTrack CVideoLoader::getTrajectoryInRange(long startFrame, long endFrame)
{
	MotionTrack empty;
        if (!m_flowTracker)
            return empty;
        if (!m_flowTracker->isTrained)
            return empty;
        return m_flowTracker->getTrajectoryInRange(startFrame, endFrame);
}

MotionTrack CVideoLoader::getTrajectoryAtCurrentFrame()
{
    MotionTrack empty;
    if (!m_flowTracker)
        return empty;
    if (!m_flowTracker->isTrained)
        return empty;
    //long startFrame = max(0,currentFrameNumber-GESTURE_MIN_TRAJECTORY_LENGTH);
    return m_flowTracker->getTrajectoryAtFrame(currentFrameNumber);
}

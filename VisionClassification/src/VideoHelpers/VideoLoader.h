/***************************************************************************
 *   Vision Classification Library                                         *
 *   Copyright (C) 2010 by:                                                *
 *      Tarek Taha, CAS-UTS  <tataha@cas.edu.au>                           *
 *      Dan Maynes-Aminzade  <monzy@cs.stanford.edu>                       *
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
#ifndef VIDEOLOADER_H
#define VIDEOLOADER_H

#include "constants.h"
#include "cv.h"
#include "highgui.h"
#include "FlowTracker.h"
#include "precomp.h"

class CVideoLoader
{
public: 
    CVideoLoader();
    ~CVideoLoader();
    bool openVideoFile(const char *);
    void loadFrame(long);
    IplImage* getMotionHistory();
    void learnTrajectories();
    void convertFrame();
    IplImage* getMaskedBitmap();
    MotionTrack getTrajectoryInRange(long startFrame, long endFrame);
    MotionTrack getTrajectoryAtCurrentFrame();

    long nFrames;
    int videoX, videoY;
    int currentFrameNumber;
    bool videoLoaded;
    IplImage *copyFrame;
    // stores the current mask associated with the filter chain
    IplImage *guessMask;

private:

    // for producing masked image frame
    IplImage *maskedFrame;
    CvCapture *videoCapture;
    CvVideoWriter *videoWriter;
    IplImage *currentFrame;
    IplImage *motionHistory;
    FlowTracker *m_flowTracker;
};

#endif

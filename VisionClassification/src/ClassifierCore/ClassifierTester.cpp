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
#include "ClassifierTester.h"

ClassifierTester::ClassifierTester()
{
    CvSize size = cvSize(QUICKTEST_X*GUESSMASK_WIDTH, QUICKTEST_Y*GUESSMASK_HEIGHT);
    quickTestImage = cvCreateImage(size, IPL_DEPTH_8U, 3);
    quickTestIplImage =  cvCreateImage(size, IPL_DEPTH_8U, 3);
}

ClassifierTester::~ClassifierTester()
{
    cvReleaseImage(&quickTestImage);
    cvReleaseImage(&quickTestIplImage);
}

void ClassifierTester::TestClassifierOnVideo(Classifier *c, CVideoLoader *vl, int recognizerMode)
{
    int nTestFrames = QUICKTEST_X * QUICKTEST_Y;
    if (!vl->videoLoaded) return;
    int originalFrame = vl->currentFrameNumber;

    if (nTestFrames > vl->nFrames) nTestFrames = vl->nFrames;
    int frameskip = floor( ((float)vl->nFrames) / ((float)nTestFrames) );

    for (int i=0; i< nTestFrames; i++) {
        vl->loadFrame(i*frameskip);
        int xpos = GUESSMASK_WIDTH*(i % QUICKTEST_X);
        int ypos = GUESSMASK_HEIGHT*(i / QUICKTEST_X);
        CvRect rect = cvRect(xpos, ypos, GUESSMASK_WIDTH, GUESSMASK_HEIGHT);
        cvSetImageROI(quickTestImage, rect);
        runClassifierOnCurrentFrame(c, vl, recognizerMode);
    }

    cvResetImageROI(quickTestImage);
    // Restore original frame position
    vl->loadFrame(originalFrame);
}

void ClassifierTester::runClassifierOnCurrentFrame(Classifier *classifier, CVideoLoader *vl, int recognizerMode)
{

    ClassifierOutputData outdata;

    if (recognizerMode == MOTION_FILTER)
    {
        outdata = ((MotionClassifier*)classifier)->classifyMotion(vl->getMotionHistory(), MOTION_NUM_HISTORY_FRAMES);
    }
    else if (recognizerMode == GESTURE_FILTER)
    {
        MotionTrack mt = vl->getTrajectoryAtCurrentFrame();
        outdata = ((GestureClassifier*)classifier)->classifyTrack(mt);
    }
    else
    {
        outdata = classifier->classifyFrame(vl->copyFrame);
    }

    IplImage *displaySmall = cvCreateImage(cvSize(GUESSMASK_WIDTH, GUESSMASK_HEIGHT), IPL_DEPTH_8U, 3);
    IplImage *displayMasked = cvCreateImage(cvSize(GUESSMASK_WIDTH, GUESSMASK_HEIGHT), IPL_DEPTH_8U, 3);
    cvZero(displayMasked);
    if (outdata.hasVariable("Mask"))
    {
        IplImage *mask = outdata.getImageData("Mask");
        cvResize(vl->copyFrame, displaySmall);
        cvCopy(displaySmall, displayMasked, mask);
    }
    cvAddWeighted(displaySmall, 0.5, displayMasked, 0.5, 0, quickTestImage);

    cvReleaseImage(&displaySmall);
    cvReleaseImage(&displayMasked);
}

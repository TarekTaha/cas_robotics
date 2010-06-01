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
#include "iostream"
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include "BrightnessClassifier.h"
#include "TrainingSet.h"
#include "Graphics.h"
int main()
{
    Graphics graphics;
    TrainingSet trainingSet;
    TrainingSample *sample1 = new TrainingSample((char*)"brightness01.png",GROUPID_POSSAMPLES);
    TrainingSample *sample2 = new TrainingSample((char*)"brightness01.png",GROUPID_POSSAMPLES);
    TrainingSample *sample3 = new TrainingSample((char*)"brightness01.png",GROUPID_POSSAMPLES);

    trainingSet.addSample(sample1);
    trainingSet.addSample(sample2);
    trainingSet.addSample(sample3);

    IplImage* frame = NULL,*guessMask=NULL;
    CvCapture* cap = NULL;
    int frameX =640 , frameY=480;
    cap   = cvCreateCameraCapture(0);
    if(cap)
    {
        cvSetCaptureProperty(cap,CV_CAP_PROP_FRAME_WIDTH,640);
        cvSetCaptureProperty(cap,CV_CAP_PROP_FRAME_HEIGHT,480);
    }
    else
    {
        std::cout<<"\nCouldnt Find a camera";
        return 0;
    }
    frame = cvQueryFrame(cap);
    if(!frame)
    {
        printf("Cant find a camera \n");
        exit(0);
    }
    BrightnessClassifier brightnessClassifier;
    brightnessClassifier.startTraining(&trainingSet);

    IplImage * copyFrame         = cvCreateImage(cvSize(frameX,frameY),IPL_DEPTH_8U,3);
    IplImage * outputFrame       = cvCreateImage(cvSize(frameX,frameY),IPL_DEPTH_8U,3);
    IplImage * outputAccImage    = cvCreateImage(cvSize(frameX,frameY),IPL_DEPTH_8U,3);
    IplImage * contourMask       = cvCreateImage(cvSize(GUESSMASK_WIDTH, GUESSMASK_HEIGHT), IPL_DEPTH_8U, 1);
    IplImage * combineMaskOutput = cvCreateImage(cvSize(GUESSMASK_WIDTH, GUESSMASK_HEIGHT), IPL_DEPTH_8U, 1);

    int  nCurrentFilter = 1;
    //create windows to show background and foreground images
    cvNamedWindow("Classifier Mask", 1);
    cvNamedWindow("Foreground", 1);
    for(;;)
    {
        frame = cvQueryFrame(cap);
        ClassifierOutputData outputData= brightnessClassifier.classifyFrame(frame);
        cvShowImage("Foreground", frame);

        if (outputData.hasVariable("Mask"))
        {
            guessMask = outputData.getImageData("Mask");
        }

        CvSeq * contours;
        if (outputData.hasVariable("Contours"))
        {
            contours = outputData.getSequenceData("Contours");
            if (contours != NULL)
            {
                cvZero(contourMask);
                cvDrawContours(contourMask, contours, cvScalar(0xFF), cvScalar(0x00), 1, 1, CV_AA);
                cvResize(contourMask, guessMask);
                cvSet(outputAccImage, colorSwatch[nCurrentFilter%COLOR_SWATCH_SIZE], guessMask);
            }
        }
        graphics.drawImage(outputAccImage);
        cvShowImage("Classifier Mask", guessMask);
        if( cvWaitKey( 10 ) >= 0 )
            break;
    }
    //release capture
    cvReleaseCapture(&cap);
    cvReleaseImage(&copyFrame);
    cvReleaseImage(&outputFrame);
    cvReleaseImage(&outputAccImage);
    cvReleaseImage(&contourMask);
    cvReleaseImage(&combineMaskOutput);
    return 0;
}

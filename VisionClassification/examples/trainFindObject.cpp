
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
#include "BackgroundSubtraction.h"
#include "SiftClassifier.h"
#include "TrainingSet.h"
#include "Graphics.h"
#include "ColorClassifier.h"
#include "ShapeClassifier.h"

int main()
{
    IplImage* frame = NULL,*guessMask=NULL;
    CvCapture* cap = NULL;
    CvFont font;
    int frameX =640 , frameY=480;
    cap   = cvCreateCameraCapture(0);
    if(cap)
    {
        cvSetCaptureProperty(cap,CV_CAP_PROP_FRAME_WIDTH,frameX);
        cvSetCaptureProperty(cap,CV_CAP_PROP_FRAME_HEIGHT,frameY);
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
    cvInitFont(&font,CV_FONT_HERSHEY_COMPLEX,0.7,0.7,0,1);
    cvNamedWindow("Classifier Mask", 1);
    cvNamedWindow("Classifier Out", 1);
    cvNamedWindow("Foreground", 1);

/*
     BackgroundSubtraction backgroundSubrationClassifier;
    int numFrames=0;
    const int  numClassificationFrames = 50;
    // Use the first "numClassificationFrames" frames to train the color classifier
    for(;(numFrames++)<numClassificationFrames;)
    {
        frame = cvQueryFrame(cap);
        ClassifierOutputData outputData= backgroundSubrationClassifier.classifyFrame(frame);
        if(!backgroundSubrationClassifier.isTrained)
        {
            char txt[200];
            sprintf(txt,"Training background subtraction, frame:%d",numFrames);
            cvPutText(frame,txt,cvPoint(50,50),&font,cvScalar(255,255,0));
            cvShowImage("Foreground", frame);
            //sleep for 100 msec
            cvWaitKey( 100 );
            continue;
        }
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
                cvDrawContours(guessMask, contours, cvScalar(0xFF), cvScalar(0x00), 1, 1, CV_AA);
            }
        }

        vector<CvRect>* boxes;
        if (outputData.hasVariable("BoundingBoxes"))
        {
            boxes = outputData.getBoundingBoxData("BoundingBoxes");
            // Use only the largest box for training
            double maxArea=0,area;
            int maxIndex=0;
            cout<<"\nNumber of boxes:"<<(*boxes).size();fflush(stdout);
            if((*boxes).size()!=0)
            {
                for(uint j=0; j< (*boxes).size();j++)
                {
                    area = (*boxes)[j].width*(*boxes)[j].height;
                    if(area>maxArea)
                    {
                        maxArea  = area;
                        maxIndex = j;
                    }
                }
                // Draw the largest rect on the original frame
                CvPoint p1 = cvPoint((*boxes)[maxIndex].x,(*boxes)[maxIndex].y);
                CvPoint p2 = cvPoint((*boxes)[maxIndex].x + (*boxes)[maxIndex].width,(*boxes)[maxIndex].y + (*boxes)[maxIndex].height);
                double xScale = frame->width/guessMask->width;
                double yScale = frame->height/guessMask->height;
                cvDrawRect(guessMask,p1,p2,cvScalar(255,255,0),1);
                CvRect scaledRect;
                scaledRect.x = (*boxes)[maxIndex].x * xScale;
                scaledRect.y = (*boxes)[maxIndex].y * yScale;
                scaledRect.width  = (*boxes)[maxIndex].width  * xScale;
                scaledRect.height = (*boxes)[maxIndex].height * yScale;
                TrainingSample * sample = new TrainingSample(frame,scaledRect,GROUPID_POSSAMPLES);
                trainingSet.addSample(sample);
                char filename[200];
                sprintf(filename,"frame-%d.png",numFrames);
                cvSetImageROI(frame,scaledRect);
                cvSaveImage(filename,frame);
                cvResetImageROI(frame);
                cvDrawRect(frame,cvPoint(scaledRect.x,scaledRect.y),cvPoint(scaledRect.x+scaledRect.width,scaledRect.y+scaledRect.height),cvScalar(255,255,0),1);
            }
        }
            //Show the frame number
        char txt[200];
        sprintf(txt,"Collecting Training samples, frame:%d",numFrames);
        cvPutText(frame,txt,cvPoint(50,50),&font,cvScalar(255,255,0));
        cvShowImage("Foreground", frame);
        cvShowImage("Classifier Mask", guessMask);
        if( cvWaitKey( 100 ) >= 0 )
            break;
    }
*/
    TrainingSet trainingSet,shapeTrainingSet;
    //TrainingSample *sample1 = new TrainingSample((char*)"colormodel.png",GROUPID_POSSAMPLES);
    //trainingSet.addSample(sample1);

    //ColorClassifier colorClassifier;
    //colorClassifier.startTraining(&trainingSet);

    //SiftClassifier siftClassifier;
    //siftClassifier.startTraining(&trainingSet);
    TrainingSample *sample2 = new TrainingSample((char*)"can.png",GROUPID_POSSAMPLES);
    TrainingSample *sample3 = new TrainingSample((char*)"can2.png",GROUPID_POSSAMPLES);
    shapeTrainingSet.addSample(sample2);
    shapeTrainingSet.addSample(sample3);

    ShapeClassifier shapeClassifier;
    shapeClassifier.startTraining(&shapeTrainingSet);
    //shapeClassifier.save();
    //siftClassifier.startTraining(&shapeTrainingSet);

    IplImage * outputAccImage    = cvCreateImage(cvSize(GUESSMASK_WIDTH,GUESSMASK_HEIGHT),IPL_DEPTH_8U,3);
    IplImage * contourMask       = cvCreateImage(cvSize(GUESSMASK_WIDTH, GUESSMASK_HEIGHT), IPL_DEPTH_8U, 1);

    int  nCurrentFilter = 1;
    for(;;)
    {
        frame = cvQueryFrame(cap);
        ClassifierOutputData outputData = shapeClassifier.classifyFrame(frame);
        //ClassifierOutputData outputData= colorClassifier.classifyFrame(frame);
        //ClassifierOutputData outputData= siftClassifier.classifyFrame(frame);
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
        //cvShowImage("Classifier Mask", colorClassifier.getApplyImage());
        cvShowImage("Classifier Out", shapeClassifier.getApplyImage());
        cvShowImage("Classifier Mask",guessMask);
        if( cvWaitKey( 10 ) >= 0 )
            break;
    }

    //release capture
    cvReleaseCapture(&cap);
    cvReleaseImage(&outputAccImage);
    cvReleaseImage(&contourMask);
    return 0;
}

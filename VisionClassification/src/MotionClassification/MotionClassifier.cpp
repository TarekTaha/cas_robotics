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
#include "MotionClassifier.h"

MotionClassifier::MotionClassifier() :
        Classifier()
{
    motionAngles.clear();

    // set the default "friendly name" and type
    strcat(friendlyName, "Motion Recognizer");
    classifierType = MOTION_FILTER;        
    // append identifier to directory name
    strcat(directoryName, FILE_MOTION_SUFFIX);
}

MotionClassifier::MotionClassifier(const char * pathname) :
    Classifier(pathname)
{
    // load the motion directions from the data file
    char filename[MAX_PATH];
    strcpy(filename, pathname);
    strcat(filename, FILE_DATA_NAME);
    FILE *datafile = fopen(filename, "rb");
    int numAngles;
    fread(&numAngles, sizeof(int), 1, datafile);
    motionAngles.clear();
    for(int i = 0; i < numAngles; i++)
    {
        double angle;
        fread(&angle, sizeof(double), 1, datafile);
        motionAngles.push_back(angle);
    }
    fclose(datafile);
    // set the type
    classifierType = MOTION_FILTER;
}

MotionClassifier::~MotionClassifier() {
}

bool MotionClassifier::containsSufficientSamples(TrainingSet *sampleSet) {
    return (sampleSet->motionSampleCount > 0);
}

void MotionClassifier::startTraining(TrainingSet *sampleSet)
{
    // Make a copy of the set used for training (we'll want to save it later)
    sampleSet->copyTo(&trainSet);

    // clear list of motion directions
    motionAngles.clear();

    IplImage *filterImageMotion = cvCloneImage(filterImage);
    IplImage *filterImageArrows = cvCloneImage(filterImage);
    cvZero(filterImage);
    cvZero(filterImageMotion);
    cvZero(filterImageArrows);

    // TODO: call into trainingset class to do this instead of accessing samplemap
    for (map<uint, TrainingSample*>::iterator i = sampleSet->sampleMap.begin(); i != sampleSet->sampleMap.end(); i++) {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_MOTIONSAMPLES) { // motion sample
            if (sample->motionHistory == NULL) {
                // no motion history associated with this sample so we skip to the next one
                continue;
            }
            CvSize size = cvSize(sample->motionHistory->width,sample->motionHistory->height);
            IplImage *orient = cvCreateImage(size, IPL_DEPTH_32F, 1);
            IplImage *segmask = cvCreateImage(size, IPL_DEPTH_32F, 1);
            IplImage *mask = cvCreateImage(size, IPL_DEPTH_8U, 1);
            IplImage *dst = cvCreateImage(size, IPL_DEPTH_8U, 3);
            CvMemStorage *storage = cvCreateMemStorage(0);

            // convert MHI to blue 8U image
            cvCvtScale(sample->motionHistory, mask, 255./MOTION_MHI_DURATION,(MOTION_MHI_DURATION-MOTION_NUM_HISTORY_FRAMES)*255./MOTION_MHI_DURATION);
            cvZero( dst );
            cvCvtPlaneToPix( mask, 0, 0, 0, dst );

            // calculate motion gradient orientation and valid orientation mask
            cvCalcMotionGradient(sample->motionHistory, mask, orient, MOTION_MAX_TIME_DELTA, MOTION_MIN_TIME_DELTA, 3 );
            
            // segment motion: get sequence of motion components
            // Segmask is marked motion components map.  It is not used further.
            CvSeq *seq = cvSegmentMotion(sample->motionHistory, segmask, storage, MOTION_NUM_HISTORY_FRAMES, MOTION_MAX_TIME_DELTA );

            //			int motion_min_component_area = 50+threshold*100.0;
            int motion_min_component_area = 100.0;
            // iterate through the motion components
            for(int i = 0; i < seq->total; i++) {
                CvRect comp_rect = ((CvConnectedComp*)cvGetSeqElem(seq, i ))->rect;
                if(comp_rect.width * comp_rect.height < motion_min_component_area ) // reject very small components
                    continue;

                // ignore motion components that are centered outside of our selection rectangle for this sample
                CvPoint centerPt = cvPoint((comp_rect.x + comp_rect.width/2), (comp_rect.y + comp_rect.height/2));
                if(!contains(sample->selectBounds,centerPt))
                    continue;
                // select component ROI
                cvSetImageROI(sample->motionHistory, comp_rect);
                cvSetImageROI(orient, comp_rect);
                cvSetImageROI(mask, comp_rect);

                // calculate orientation
                double motionAngle = cvCalcGlobalOrientation( orient, mask, sample->motionHistory, 1.0, MOTION_MHI_DURATION);
                //				motionAngle = 360.0 - motionAngle;  // adjust for images with top-left origin
                motionAngles.push_back(motionAngle);

                cvResetImageROI(sample->motionHistory);
                cvResetImageROI(orient);
                cvResetImageROI(mask);

                // draw the motion arrow for this direction into the demo image
                CvScalar color = CV_RGB(255,255,255);
                double magnitude = min(filterImage->width/3, filterImage->height/3);
                CvPoint center = cvPoint(filterImage->width/2,filterImage->height/2);
                cvCircle(filterImageArrows, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
                if(graphics)
                    graphics->drawArrow(filterImageArrows, center, motionAngle, magnitude, color, 3);
            }

            // copy the motion in this sample to the demo image
            cvSetImageROI(dst, cvRect(sample->selectBounds.x, sample->selectBounds.y,
                                      sample->selectBounds.width, sample->selectBounds.height));
            cvResize(dst, filterImageMotion);
            cvAddWeighted(filterImageMotion, 1.0/((float)sampleSet->motionSampleCount), filterImage, 1.0, 0, filterImage);
            cvResetImageROI(dst);

            cvReleaseImage(&orient);
            cvReleaseImage(&segmask);
            cvReleaseImage(&mask);
            cvReleaseImage(&dst);
            cvReleaseMemStorage(&storage);

        }
    }

    // copy arrows to demo image
    cvAdd(filterImageArrows, filterImage, filterImage);
    cvReleaseImage(&filterImageMotion);
    cvReleaseImage(&filterImageArrows);

    if (isOnDisk) { // this classifier has been saved so we'll update the files
        save();        
    }

    // update member variables
    isTrained = true;
}

ClassifierOutputData MotionClassifier::classifyFrame(IplImage *frame) {
    // not implemented: this classifier uses classifyMotion instead
    assert(false);
	ClassifierOutputData data;
	return data;
}

ClassifierOutputData MotionClassifier::classifyMotion(IplImage *frame, double timestamp)
{
    cvZero(guessMask);
    if (!isTrained) return outputData;
    if(!frame) return outputData;

    // check to make sure that the frame passed in is a motion history image (not a normal frame image)
    if (frame->depth != IPL_DEPTH_32F) return outputData;
    if (frame->nChannels != 1) return outputData;

    // first find the motion components in this motion history image
    CvSize size = cvSize(frame->width,frame->height);
    IplImage *orient = cvCreateImage(size, IPL_DEPTH_32F, 1);
    IplImage *segmask = cvCreateImage(size, IPL_DEPTH_32F, 1);
    IplImage *mask = cvCreateImage(size, IPL_DEPTH_8U, 1);
    IplImage *dst = cvCreateImage(size, IPL_DEPTH_8U, 3);
    IplImage *newMask = cvCreateImage(size, IPL_DEPTH_8U, 1);
    cvZero(newMask);
    CvMemStorage *storage = cvCreateMemStorage(0);

    // convert MHI to blue 8U image
    cvCvtScale(frame, mask, 255./MOTION_MHI_DURATION,(MOTION_MHI_DURATION-MOTION_NUM_HISTORY_FRAMES)*255./MOTION_MHI_DURATION);
    cvZero( dst );
    cvCvtPlaneToPix( mask, 0, 0, 0, dst );

    // calculate motion gradient orientation and valid orientation mask
    cvCalcMotionGradient(frame, mask, orient, MOTION_MAX_TIME_DELTA, MOTION_MIN_TIME_DELTA, 3);
    
    // segment motion: get sequence of motion components
    // Segmask is marked motion components map.  It is not used further.
    CvSeq *seq = cvSegmentMotion(frame, segmask, storage, timestamp, MOTION_MAX_TIME_DELTA);

    int motion_min_component_area = 50+threshold*100.0;

    // iterate through the motion components
    for(int i = 0; i < seq->total; i++) {
        CvRect comp_rect = ((CvConnectedComp*)cvGetSeqElem(seq, i ))->rect;
        if(comp_rect.width * comp_rect.height < motion_min_component_area ) // reject very small components
            continue;
        double magnitude = 30;

        // select component ROI
        cvSetImageROI(frame, comp_rect );
        cvSetImageROI(orient, comp_rect );
        cvSetImageROI(mask, comp_rect );

        // calculate orientation
        double motionAngle = cvCalcGlobalOrientation(orient, mask, frame, MOTION_NUM_HISTORY_FRAMES, MOTION_MHI_DURATION);
        //        motionAngle = 360.0 - motionAngle;  // adjust for images with top-left origin

        cvResetImageROI(frame);
        cvResetImageROI(orient);
        cvResetImageROI(mask);

        // draw mogion components in red unless there is a motion direction match
        CvScalar color = CV_RGB(255,100,100);

        int angle_diff_threshold = 60-threshold*58.0;

        // check if the direction of this component matches the direction in one of the samples
        for (list<double>::iterator i = motionAngles.begin(); i!=motionAngles.end(); i++) {
            double angleDiff = fabs((*i)-motionAngle);
            // for angles close to 360
            if (angleDiff > (360-angle_diff_threshold)) angleDiff = 360-angleDiff;

            if (angleDiff < angle_diff_threshold) { // add to list of guesses
                color = CV_RGB(255,255,255);

                // draw rectangle in mask image
                cvRectangle(newMask, cvPoint(comp_rect.x, comp_rect.y),
                            cvPoint(comp_rect.x+comp_rect.width, comp_rect.y+comp_rect.height),
                            cvScalar(0xFF), CV_FILLED, 8);
            }
        }
        // draw a clock with arrow indicating the direction
        CvPoint center = cvPoint((comp_rect.x + comp_rect.width/2), (comp_rect.y + comp_rect.height/2));

        cvCircle(dst, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
        if(graphics)
            graphics->drawArrow(dst, center, motionAngle, magnitude, color, 3);
    }
    // copy motion picture to demo image
    cvResize(dst, applyImage);

    // copy the final output mask
    cvResize(newMask, guessMask);

    cvReleaseImage(&orient);
    cvReleaseImage(&segmask);
    cvReleaseImage(&mask);
    cvReleaseImage(&dst);
    cvReleaseImage(&newMask);
    cvReleaseMemStorage(&storage);

    updateStandardOutputData();
    return outputData;
}

void MotionClassifier::save()
{
    if (!isTrained) return;

    Classifier::save();

    char filename[MAX_PATH];

    // save the motion angle data
    strcpy(filename,directoryName);
    strcat(filename, FILE_DATA_NAME);
    FILE *datafile = fopen(filename, "wb");

    int numAngles = motionAngles.size();
    fwrite(&numAngles, sizeof(int), 1, datafile);
    for(list<double>::iterator i = motionAngles.begin(); i != motionAngles.end(); i++)
    {
        double angle = (*i);
        fwrite(&angle, sizeof(double), 1, datafile);
    }
    fclose(datafile);
}

void MotionClassifier::setGraphics(Graphics *_graphics)
{
    this->graphics = _graphics;
}

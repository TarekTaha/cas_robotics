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
#include "ColorClassifier.h"

ColorClassifier::ColorClassifier() :
        Classifier()
{
    // allocate histogram
    hdims = 16;
    float hranges_arr[2];
    hranges_arr[0] = 0;	hranges_arr[1] = 180;
    float* hranges = hranges_arr;
    hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );

    // set the default "friendly name" and type
    strcpy(friendlyName, "Color Recognizer");
    classifierType = COLOR_FILTER;        
    
    // append identifier to directory name
    strcat(directoryName, FILE_COLOR_SUFFIX);
}

ColorClassifier::ColorClassifier(const char * pathname) :
        Classifier(pathname)
{
    // set the type
    classifierType = COLOR_FILTER;
    // allocate histogram
    hdims = 16;
    float hranges_arr[2];
    hranges_arr[0] = 0;	hranges_arr[1] = 180;
    float* hranges = hranges_arr;
    hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );

    char filename[MAX_PATH];
    sprintf(filename,"%s/%s",directoryName,classifierDataFileName);

    // load the data from the histogram file
    load(filename);
}

ColorClassifier::~ColorClassifier()
{
    // free histogram
    cvReleaseHist(&hist);
}

void ColorClassifier::load(const char * fileName)
{
    // load the data from the histogram file
    FILE *datafile = fopen(fileName, "rb");
    for(int i = 0; i < hdims; i++)
    {
        float val;
        fread(&val, sizeof(float), 1, datafile);
        cvSetReal1D(hist->bins,i,val);
    }
    fclose(datafile);
    isTrained = true;
    updateHistogramImage();
}

void ColorClassifier::load(string fileName)
{
    load(fileName.c_str());
}

bool ColorClassifier::containsSufficientSamples(TrainingSet *sampleSet)
{
    return (sampleSet->posSampleCount > 0);
}

void ColorClassifier::startTraining(TrainingSet *sampleSet)
{
    // just to filter empty sets (and YES it happens)
    if(sampleSet->posSampleCount<1)
        return;
    // Make a copy of the set used for training (we'll want to save it later)
    sampleSet->copyTo(&trainSet);

    // clear out the histogram
    cvClearHist(hist);

    // TODO: call into trainingset class to do this instead of accessing samplemap
    for (map<uint, TrainingSample*>::iterator i = sampleSet->sampleMap.begin(); i != sampleSet->sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        // positive sample
        if (sample->iGroupId == GROUPID_POSSAMPLES)
        {

            // allocate image buffers
            IplImage *hsv = cvCreateImage( cvGetSize(sample->fullImageCopy), 8, 3 );
            IplImage *hue = cvCreateImage( cvGetSize(sample->fullImageCopy), 8, 1 );
            IplImage *mask = cvCreateImage( cvGetSize(sample->fullImageCopy), 8, 1 );

            // convert to hsv space
            cvCvtColor(sample->fullImageCopy, hsv, CV_BGR2HSV);

            // clip max and min range and split out hue channel
            cvInRangeS(hsv, cvScalar(0,COLOR_SMIN,COLOR_VMIN,0),cvScalar(180,256,COLOR_VMAX,0), mask);
            cvSplit(hsv, hue, 0, 0, 0);

            // accumulate into hue histogram
            cvCalcHist(&hue, hist, 1, mask);

            // free image buffers
            cvReleaseImage(&hsv);
            cvReleaseImage(&hue);
            cvReleaseImage(&mask);

        }
        // negative sample
        else if (sample->iGroupId == GROUPID_NEGSAMPLES)
        {
            // TODO: we could potentially subtract this from histogram
        }
    }

    updateHistogramImage();

    if (isOnDisk)
    { // this classifier has been saved so we'll update the files
        save();
    }

    // update member variables
    isTrained = true;
}

ClassifierOutputData ColorClassifier::classifyFrame(IplImage *frame)
{
    cvZero(guessMask);
    if (!isTrained)
        return outputData;
    if(!frame)
        return outputData;
    IplImage *image = cvCreateImage( cvGetSize(frame), 8, 3 );
    IplImage *hsv = cvCreateImage( cvGetSize(frame), 8, 3 );
    IplImage *hue = cvCreateImage( cvGetSize(frame), 8, 1 );
    IplImage *mask = cvCreateImage( cvGetSize(frame), 8, 1 );
    IplImage *backproject = cvCreateImage( cvGetSize(frame), 8, 1 );
    IplImage *newMask = cvCreateImage( cvGetSize(frame), 8, 1 );
    cvZero(newMask);

    cvCopy( frame, image, 0 );
    cvCvtColor( image, hsv, CV_BGR2HSV );

    // create mask to clip out pixels outside of specified range
    cvInRangeS(hsv, cvScalar(0,COLOR_SMIN,COLOR_VMIN,0), cvScalar(180,256,COLOR_VMAX,0), mask);
    cvSplit(hsv, hue, 0, 0, 0 );

    // create backprojection image and clip with mask
    cvCalcBackProject(&hue, backproject, hist);
    cvAnd(backproject, mask, backproject, 0);

    // copy back projection into demo image
    cvCvtColor(backproject, image, CV_GRAY2BGR);

    // create contour storage
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contours = NULL;

    // threhold the backprojection image
    cvThreshold(backproject, backproject, threshold*255, 255, CV_THRESH_BINARY);

    // close the backprojection image
    IplConvKernel *circElem = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_ELLIPSE);        
    cvMorphologyEx(backproject, backproject, 0, circElem, CV_MOP_CLOSE, 1);  
    cvReleaseStructuringElement(&circElem);

    // find contours in backprojection image
    cvFindContours( backproject, storage, &contours, sizeof(CvContour),
                    CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

    // Loop over the found contours
    for (; contours != NULL; contours = contours->h_next)
    {
        double contourArea = fabs(cvContourArea(contours));
        if ((contourArea > COLOR_MIN_AREA) && (contourArea < COLOR_MAX_AREA))
        {

            // draw contour in new mask image
            cvDrawContours(newMask, contours, cvScalar(0xFF), cvScalar(0xFF), 0, CV_FILLED, 8);

            // draw contour in demo image
            cvDrawContours(image, contours, CV_RGB(0,255,255), CV_RGB(0,255,255), 0, 2, 8);
        }
    }

    // copy the final output mask
    cvResize(newMask, guessMask);

    // update bitmap demo image
    cvResize(image, applyImage);
    cvReleaseMemStorage(&storage);

    cvReleaseImage(&image);
    cvReleaseImage(&hsv);
    cvReleaseImage(&hue);
    cvReleaseImage(&mask);
    cvReleaseImage(&backproject);
    cvReleaseImage(&newMask);

    updateStandardOutputData();
    return outputData;
}

void ColorClassifier::updateHistogramImage()
{

    // create histogram image
    IplImage *histimg = cvCreateImage( cvSize(320,200), 8, 3 );
    float max_val = 0.f;
    cvGetMinMaxHistValue( hist, 0, &max_val, 0, 0 );
    cvConvertScale( hist->bins, hist->bins, max_val ? 255. / max_val : 0., 0 );
    cvZero( histimg );
    int bin_w = histimg->width / hdims;
    for(int i = 0; i < hdims; i++)
    {
        int val = cvRound( cvGetReal1D(hist->bins,i)*histimg->height/255 );
        CvScalar color = hsv2rgb(i*180.f/hdims);
        cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
                     cvPoint((i+1)*bin_w,histimg->height - val),
                     color, -1, 8, 0 );
    }
    cvResize(histimg, filterImage);
    cvReleaseImage(&histimg);
}

void ColorClassifier::save()
{
    if (!isTrained) 
        return;

    Classifier::save();
    char filename[MAX_PATH];

    sprintf(filename,"%s/%s",directoryName,classifierDataFileName);

    FILE *datafile = fopen(filename, "wb");
    for(int i = 0; i < hdims; i++)
    {
        float val = cvGetReal1D(hist->bins,i);
        fwrite(&val, sizeof(float), 1, datafile);
    }
    fclose(datafile);
}

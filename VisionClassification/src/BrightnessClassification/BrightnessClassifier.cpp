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
#include "BrightnessClassifier.h"

BrightnessClassifier::BrightnessClassifier() :
        Classifier()
{
    // allocate histogram
    hdims = 16;
    float hranges_arr[2];
    hranges_arr[0] = 0;	hranges_arr[1] = 255;
    float* hranges = hranges_arr;
    hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );

    // set the default "friendly name" and type
    sprintf(friendlyName,"%s", "Brightness Filter");
    classifierType = BRIGHTNESS_FILTER;
    
    // append identifier to directory name
    strcat(directoryName, FILE_BRIGHTNESS_SUFFIX);
}

BrightnessClassifier::BrightnessClassifier(const char* pathname) :
        Classifier(pathname)
{
    // allocate histogram
    hdims = 16;
    float hranges_arr[2];
    hranges_arr[0] = 0;	hranges_arr[1] = 255;
    float* hranges = hranges_arr;
    hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );

    char filename[MAX_PATH];
    strcpy(filename, pathname);
    strcat(filename, FILE_DATA_NAME);

    // load the data from the histogram file (and compute average level)
    FILE *datafile = fopen(filename, "rb");
    avg_level = 0;
    for(int i = 0; i < hdims; i++)
    {
        float val;
        fread(&val, sizeof(float), 1, datafile);
        cvSetReal1D(hist->bins,i,val);
        avg_level += val;
    }
    avg_level /= hdims;
    fclose(datafile);

    // set the type
    classifierType = BRIGHTNESS_FILTER;

    updateHistogramImage();
}

BrightnessClassifier::~BrightnessClassifier()
{
    // free histogram
    cvReleaseHist(&hist);
}

bool BrightnessClassifier::containsSufficientSamples(TrainingSet *sampleSet)
{
    return (sampleSet->posSampleCount > 0);
}

void BrightnessClassifier::startTraining(TrainingSet *sampleSet)
{
    // Make a copy of the set used for training (we'll want to save it later)
    sampleSet->copyTo(&trainSet);

    // clear out the histogram
    cvClearHist(hist);

    // TODO: call into trainingset class to do this instead of accessing samplemap
    for (map<uint, TrainingSample*>::iterator i = sampleSet->sampleMap.begin(); i != sampleSet->sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_POSSAMPLES)
        {
            // positive sample
            // create grayscale copy
            IplImage *brightness = cvCreateImage( cvGetSize(sample->fullImageCopy), IPL_DEPTH_8U, 1);
            cvCvtColor(sample->fullImageCopy, brightness, CV_BGR2GRAY);

            // accumulate into hue histogram
            cvCalcHist(&brightness, hist, 1);

            // free grayscale copy
            cvReleaseImage(&brightness);

        }
        else if (sample->iGroupId == GROUPID_NEGSAMPLES)
        {
            // negative sample
            // TODO: we could potentially subtract this from histogram
        }
    }

    updateHistogramImage();

    if (isOnDisk)
    {
        // this classifier has been saved so we'll update the files
        save();        
    }

    // update member variables
    isTrained = true;
}

ClassifierOutputData BrightnessClassifier::classifyFrame(IplImage *frame)
{
    cvZero(guessMask);
    if (!isTrained)
        return outputData;
    if(!frame)
        return outputData;

    IplImage *image = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 3 );
    IplImage *brightness = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
    IplImage *backproject = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
    IplImage *newMask = cvCreateImage( cvGetSize(frame), IPL_DEPTH_8U, 1 );
    cvZero(newMask);

    cvCopy(frame, image, 0);
    cvCvtColor(image, brightness, CV_BGR2GRAY);

    // create backprojection image
    cvCalcBackProject(&brightness, backproject, hist);

    // copy back projection into demo image
    cvCvtColor(backproject, image, CV_GRAY2BGR);

    // create contour storage
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contours = NULL;

    // threhold the backprojection image
    cvThreshold(backproject, backproject, threshold*255, 255, CV_THRESH_BINARY);

    // close the backprojection image
    IplConvKernel *circElem = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_ELLIPSE);        
    cvMorphologyEx(backproject, backproject, 0, circElem, CV_MOP_CLOSE, 2);
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

    // update IplImage* demo image
    cvResize(image, applyImage);
    cvCopyImage(applyImage, applyIplImage);

    cvReleaseMemStorage(&storage);

    cvReleaseImage(&image);
    cvReleaseImage(&brightness);
    cvReleaseImage(&backproject);
    cvReleaseImage(&newMask);

    updateStandardOutputData();
    return outputData;
}

void BrightnessClassifier::updateHistogramImage()
{
    // create histogram image
    IplImage *histimg = cvCreateImage(cvSize(320,200), 8, 3);
    float max_val = 0.f;
    cvGetMinMaxHistValue(hist, 0, &max_val, 0, 0 );
    cvConvertScale(hist->bins, hist->bins, max_val ? 255./max_val : 0., 0);
    cvSet(histimg, CV_RGB(220,220,240));
    int bin_w = histimg->width / hdims;
    avg_level = 0;
    float total_size = 0;
    for(int i = 0; i < hdims; i++)
    {
        double bin_size = cvGetReal1D(hist->bins,i);
        double bin_level = i*255.0f/hdims;
        int val = cvRound( bin_size*histimg->height/255 );
        avg_level += bin_size*bin_level;
        total_size += bin_size;

        CvScalar color = CV_RGB(i*255.0f/hdims,i*255.0f/hdims,i*255.0f/hdims);
        cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
                     cvPoint((i+1)*bin_w,histimg->height - val),
                     color, -1, 8, 0 );
        cvRectangle( histimg, cvPoint(i*bin_w,histimg->height),
                     cvPoint((i+1)*bin_w,histimg->height - val),
                     CV_RGB(0,0,0), 1, 8, 0 );
    }
    avg_level /= total_size;

    // draw a red line at the average level
    cvLine( histimg, cvPoint(0, histimg->height - avg_level*histimg->height/255),
            cvPoint(histimg->width, histimg->height - avg_level*histimg->height/255),
            CV_RGB(255,0,0), 2, 8, 0);

    cvResize(histimg, filterImage);
    cvCopyImage(filterImage,filterIplImage);
    cvReleaseImage(&histimg);
}

void BrightnessClassifier::save()
{
    if (!isTrained)
        return;

    Classifier::save();
    char filename[MAX_PATH];

    // save the histogram data
    strcat(filename,directoryName);
    strcat(filename, FILE_DATA_NAME);
    FILE *datafile = fopen(filename, "wb");
    for(int i = 0; i < hdims; i++)
    {
        float val = cvGetReal1D(hist->bins,i);
        fwrite(&val, sizeof(float), 1, datafile);
    }
    fclose(datafile);
}

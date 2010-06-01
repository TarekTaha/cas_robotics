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
#include "CamshiftClassifier.h"

CamshiftClassifier::CamshiftClassifier() :
        Classifier()
{
    image = NULL;
    hsv = NULL;
    hue = NULL;
    mask = NULL;
    backproject = NULL;
    histimg = NULL;
    hist = NULL;

    hdims = 16;
    hranges_arr[0] = 0;	hranges_arr[1] = 180;
    hranges = hranges_arr;

    // allocate histogram
    hist = cvCreateHist( 1, &hdims, CV_HIST_ARRAY, &hranges, 1 );
}

CamshiftClassifier::~CamshiftClassifier()
{
    // free histogram
    cvReleaseHist(&hist);
}

void CamshiftClassifier::startTraining(TrainingSet *sampleSet)
{
    // clear out the histogram
    cvClearHist(hist);

    // TODO: call into trainingset class to do this instead of accessing samplemap
    for (map<uint, TrainingSample*>::iterator i = sampleSet->sampleMap.begin(); i != sampleSet->sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == 0)
        {
            // positive sample

            // allocate image buffers
            hsv = cvCreateImage( cvGetSize(sample->fullImageCopy), 8, 3 );
            hue = cvCreateImage( cvGetSize(sample->fullImageCopy), 8, 1 );
            mask = cvCreateImage( cvGetSize(sample->fullImageCopy), 8, 1 );

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
        else if (sample->iGroupId == 1)
        {
            // negative sample
            // TODO: we could potentially subtract this from histogram
        }
    }

    // create histogram image
    histimg = cvCreateImage( cvSize(320,200), 8, 3 );
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

    // update member variables
    nPosSamples = sampleSet->posSampleCount;
    nNegSamples = sampleSet->negSampleCount;
    isTrained = true;
}

void CamshiftClassifier::classifyFrame(IplImage *frame, list<CvRect>* objList)
{
    if (!isTrained)
        return;
    if(!frame)
        return;

    image = cvCreateImage( cvGetSize(frame), 8, 3 );
    hsv = cvCreateImage( cvGetSize(frame), 8, 3 );
    hue = cvCreateImage( cvGetSize(frame), 8, 1 );
    mask = cvCreateImage( cvGetSize(frame), 8, 1 );
    backproject = cvCreateImage( cvGetSize(frame), 8, 1 );

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

    // find contours in backprojection image
    CvMemStorage* storage = cvCreateMemStorage(0);
    CvSeq* contours = NULL;
    cvFindContours( backproject, storage, &contours, sizeof(CvContour),
                    CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0) );

    // Loop over the found contours
    objList->clear();
    for (; contours != NULL; contours = contours->h_next)
    {
        double contourArea = fabs(cvContourArea(contours));
        if ((contourArea > COLOR_MIN_AREA) && (contourArea < COLOR_MAX_AREA))
        {
            CvRect objRect;
            CvRect rect = cvBoundingRect(contours);
            objRect.x = rect.x;
            objRect.y = rect.y;
            objRect.width = rect.width;
            objRect.height = rect.height;
            objList->push_back(objRect);

            // draw contour in demo image
            cvDrawContours(image, contours, CV_RGB(0,255,255), CV_RGB(0,255,255), 0, 2, 8);
        }
    }

    // update bitmap demo image
    cvResize(image, applyImage);
    cvReleaseMemStorage(&storage);

    cvReleaseImage(&image);
    cvReleaseImage(&hsv);
    cvReleaseImage(&hue);
    cvReleaseImage(&mask);
    cvReleaseImage(&backproject);
}

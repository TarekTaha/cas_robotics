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
#include "ShapeClassifier.h"

double pghMatchShapes(CvSeq *shape1, CvSeq *shape2)
{
    int dims[] = {8, 8};
    float range[] = {-180, 180, -100, 100};
    float *ranges[] = {&range[0], &range[2]};
    CvHistogram* hist1 = cvCreateHist(2, dims, CV_HIST_ARRAY, ranges, 1);
    CvHistogram* hist2 = cvCreateHist(2, dims, CV_HIST_ARRAY, ranges, 1);
    cvCalcPGH(shape1, hist1);
    cvCalcPGH(shape2, hist2);
    cvNormalizeHist(hist1, 100.0f);
    cvNormalizeHist(hist2, 100.0f);
    double corr = cvCompareHist(hist1, hist2, CV_COMP_BHATTACHARYYA);
    cvReleaseHist(&hist1);
    cvReleaseHist(&hist2);
    return corr;
}

ShapeClassifier::ShapeClassifier() :
        Classifier()
{
    templateStorage = cvCreateMemStorage(0);    
    // set the default "friendly name" and type
    strcpy(friendlyName, "Shape Recognizer");
    classifierType = SHAPE_FILTER;            
    // append identifier to directory name
    strcat(directoryName, FILE_SHAPE_SUFFIX);
}

ShapeClassifier::ShapeClassifier(const char *pathname) :
        Classifier(pathname)
{
    templateStorage = cvCreateMemStorage(0);

    char filename[MAX_PATH];
    strcpy(filename, pathname);
    strcat(filename, FILE_CONTOUR_NAME);

    // load the contours from the data file
    templateContours = (CvSeq*)cvLoad(filename, templateStorage, 0, 0);

    // set the type
    classifierType = SHAPE_FILTER;

    updateContourImage();
}

ShapeClassifier::~ShapeClassifier()
{
    cvReleaseMemStorage(&templateStorage);
}

bool ShapeClassifier::containsSufficientSamples(TrainingSet *sampleSet)
{
    return (sampleSet->posSampleCount > 0);
}

void ShapeClassifier::startTraining(TrainingSet *sampleSet)
{
    // Make a copy of the set used for training (we'll want to save it later)
    sampleSet->copyTo(&trainSet);

    cvClearMemStorage(templateStorage);
    templateContours = NULL;

    // TODO: call into trainingset class to do this instead of accessing samplemap
    for (map<uint, TrainingSample*>::iterator i = sampleSet->sampleMap.begin(); i != sampleSet->sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_POSSAMPLES)
        {
            // positive sample
            IplImage *grayscale = cvCreateImage( cvSize(sample->fullImageCopy->width, sample->fullImageCopy->height), IPL_DEPTH_8U, 1);
            cvCvtColor(sample->fullImageCopy, grayscale, CV_BGR2GRAY);
            cvCanny(grayscale, grayscale, SHAPE_CANNY_EDGE_LINK, SHAPE_CANNY_EDGE_FIND, SHAPE_CANNY_APERTURE);
            cvDilate(grayscale, grayscale, 0, 2);

            CvMemStorage *storage = cvCreateMemStorage(0);
            CvSeq *sampleContours = NULL;

            cvFindContours(grayscale, storage, &sampleContours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);
            if (sampleContours != NULL)
            {
                sampleContours = cvApproxPoly(sampleContours, sizeof(CvContour), storage, CV_POLY_APPROX_DP, 0.2, 1 );
                for (CvSeq *contour = sampleContours; contour != NULL; contour = contour->h_next)
                {
                    if ((contour->total > SHAPE_MIN_CONTOUR_POINTS) && (contour->flags & CV_SEQ_FLAG_CLOSED))
                    {
                        if (!templateContours)
                        {
                            templateContours = cvCloneSeq(contour, templateStorage);
                        }
                        else
                        {
                            CvSeq *newContour = cvCloneSeq(contour, templateStorage);
                            newContour->h_next = templateContours->h_next;
                            templateContours->h_next = newContour;
                        }
                    }
                }
            }
            cvReleaseMemStorage(&storage);
            cvReleaseImage(&grayscale);

        } else if (sample->iGroupId == GROUPID_NEGSAMPLES)
        {
            // negative sample
            // do nothing for now
            // TODO: we could compare guesses against these as well and remove them if they match
        }
    }

    updateContourImage();

    if (isOnDisk)
    {
        // this classifier has been saved so we'll update the files
        save();        
    }

    // update member variables
    isTrained = true;
}

ClassifierOutputData ShapeClassifier::classifyFrame(IplImage *frame)
{
    cvZero(guessMask);
    if (!isTrained)
        return outputData;
    if(!frame)
        return outputData;

    IplImage *copy = cvCreateImage( cvSize(frame->width, frame->height), IPL_DEPTH_8U, 3);
    IplImage *grayscale = cvCreateImage( cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
    IplImage *newMask = cvCreateImage( cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
    cvZero(newMask);

    cvCvtColor(frame, grayscale, CV_BGR2GRAY);
    cvCanny(grayscale, grayscale, SHAPE_CANNY_EDGE_LINK, SHAPE_CANNY_EDGE_FIND, SHAPE_CANNY_APERTURE);
    cvDilate(grayscale, grayscale, 0, 2);

    cvCvtColor(grayscale, copy, CV_GRAY2BGR);

    CvSeq *frameContours;
    CvMemStorage *storage = cvCreateMemStorage(0);
    cvFindContours(grayscale, storage, &frameContours, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_TC89_KCOS);

    for (CvSeq *contour = frameContours; contour != NULL; contour = contour->h_next)
    {
        if ( contour->total > SHAPE_MIN_CONTOUR_POINTS)
        {
            int contourNum = 0;
            for (CvSeq *matchContour = templateContours; matchContour != NULL; matchContour = matchContour->h_next)
            {
                //double match_error = cvMatchShapes(contour, matchContour, CV_CONTOURS_MATCH_I1, 0);
                double match_error = pghMatchShapes(contour, matchContour);
                if (match_error < (0.75-threshold*.75))
                {
                    cvDrawContours(copy, contour, colorSwatch[contourNum], CV_RGB(0,0,0), 0, 3, 8, cvPoint(0,0));
                    CvRect rect = cvBoundingRect(contour, 1);
                    // draw rectangle in mask image
                    cvRectangle(newMask, cvPoint(rect.x, rect.y), cvPoint(rect.x+rect.width, rect.y+rect.height), cvScalar(0xFF), CV_FILLED, 8);
                }
                contourNum = (contourNum+1) % COLOR_SWATCH_SIZE;
            }
        }
    }
    cvResize(copy, applyImage);
    // copy the final output mask
    cvResize(newMask, guessMask);

    cvReleaseMemStorage(&storage);
    cvReleaseImage(&copy);
    cvReleaseImage(&grayscale);
    cvReleaseImage(&newMask);

    updateStandardOutputData();
    return outputData;
}

void ShapeClassifier::updateContourImage()
{
    cvZero(filterImage);

    // first, determine how many template contours we need to draw by counting the length of the sequence
    int numContours = 0;
    for (CvSeq *contour = templateContours; contour != NULL; contour = contour->h_next)
    {
        numContours++;
    }
    if (numContours > 0)
    {

        int gridSize = (int) ceil(sqrt((double)numContours));
        int gridX = 0;
        int gridY = 0;
        int gridSampleW = FILTERIMAGE_WIDTH / gridSize;
        int gridSampleH = FILTERIMAGE_HEIGHT / gridSize;
        int contourNum = 0;
        for (CvSeq *contour = templateContours; contour != NULL; contour = contour->h_next)
        {

            cvSetImageROI(filterImage, cvRect(gridX*gridSampleW, gridY*gridSampleH, gridSampleW, gridSampleH));

            CvRect bounds = cvBoundingRect(contour, 1);
            int contourSize = max(bounds.width, bounds.height);
            IplImage *contourImg = cvCreateImage(cvSize(contourSize, contourSize), filterImage->depth, filterImage->nChannels);

            cvZero(contourImg);
            cvDrawContours(contourImg, contour, colorSwatch[contourNum], CV_RGB(255,255,255), 0, 2, CV_AA, cvPoint(-bounds.x, -bounds.y));
            cvResize(contourImg, filterImage);
            cvReleaseImage(&contourImg);
            cvResetImageROI(filterImage);

            contourNum = (contourNum+1) % COLOR_SWATCH_SIZE;
            gridX++;
            if (gridX >= gridSize)
            {
                gridX = 0;
                gridY++;
            }
        }
    }
}

void ShapeClassifier::save()
{
    if (!isTrained)
        return;

    Classifier::save();

    char filename[MAX_PATH];

    // save the contour data
    strcpy(filename,directoryName);
    strcat(filename, FILE_CONTOUR_NAME);

    const char* contour_attrs[] =
    {
        "recursive", "1", 0
    };
    cvSave(filename, templateContours, 0, 0, cvAttrList(contour_attrs,0));
}

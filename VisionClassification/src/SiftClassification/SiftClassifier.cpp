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
#include "SiftClassifier.h"

SiftClassifier::SiftClassifier() :
        Classifier()
{
    numSampleFeatures = 0;
    sampleCopy = NULL;
    sampleFeatures = NULL;

    // set the default "friendly name" and type
    strcpy(friendlyName, "SIFT Recognizer");
    classifierType = SIFT_FILTER;

    // append identifier to directory name
    strcat(directoryName, FILE_SIFT_SUFFIX);
}

SiftClassifier::SiftClassifier(const char * pathname) :
        Classifier(pathname)
{
    // set the type
    classifierType = SIFT_FILTER;
    numSampleFeatures = 0;
    sampleCopy = NULL;
    sampleFeatures = NULL;

    char filename[MAX_PATH];
    sprintf(filename,"%s/%s",directoryName,classifierDataFileName);
    load(filename);
}

SiftClassifier::~SiftClassifier()
{
    if (sampleCopy)
        cvReleaseImage(&sampleCopy);
    if (numSampleFeatures > 0)
        free(sampleFeatures);
}

void SiftClassifier::load(const char * fileName)
{
    // load the features from the data file
    numSampleFeatures = import_features((char*)fileName, FEATURE_LOWE, &sampleFeatures);
     char siftSampleImage[MAX_PATH];
    // load the filter sample image
    strcpy(siftSampleImage, fileName);
    strcat(siftSampleImage, FILE_SIFTIMAGE_NAME);
    sampleCopy = cvLoadImage(siftSampleImage);
    sampleWidth = sampleCopy->width;
    sampleHeight = sampleCopy->height;
    updateSiftImage();
}

void SiftClassifier::load(string fileName)
{
    load(fileName.c_str());
}

bool SiftClassifier::containsSufficientSamples(TrainingSet *sampleSet)
{
    return (sampleSet->posSampleCount > 0);
}

void SiftClassifier::startTraining(TrainingSet *sampleSet)
{
    // just to filter empty sets (and YES it happens)
    if(sampleSet->posSampleCount<1)
        return;
    // Make a copy of the set used for training (we'll want to save it later)
    sampleSet->copyTo(&trainSet);

    if (sampleCopy)
        cvReleaseImage(&sampleCopy);
    if (numSampleFeatures > 0)
        free(sampleFeatures);

    // TODO: call into trainingset class to do this instead of accessing samplemap
    for (map<uint, TrainingSample*>::iterator i = sampleSet->sampleMap.begin(); i != sampleSet->sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_POSSAMPLES)
        { // positive sample

            // store a copy of the sample image for later
            sampleCopy = cvCloneImage(sample->fullImageCopy);
            sampleWidth = sampleCopy->width;
            sampleHeight = sampleCopy->height;
            numSampleFeatures = sift_features(sample->fullImageCopy, &sampleFeatures);

            // for now just get features from first sample
            // TODO: collect feature sets from each sample?
            break;
        }
        else if (sample->iGroupId == GROUPID_NEGSAMPLES)
        {
            // negative sample
        }
    }

    if (isOnDisk)
    {
        // this classifier has been saved so we'll update the files
        save();        
    }

    // update member variables
    isTrained = true;

    updateSiftImage();
}

ClassifierOutputData SiftClassifier::classifyFrame(IplImage *frame)
{
    cvZero(guessMask);
    if (!isTrained)
        return outputData;
    if(!frame)
        return outputData;

    // copy current frame and sample image for demo image
    IplImage *frameCopy = cvCloneImage(frame);
    IplImage *featureImage = cvCloneImage(sampleCopy);
    IplImage *newMask = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
    cvZero(newMask);

    // get features in current frame
    struct feature *frameFeatures;
    int nFeatures = sift_features(frameCopy, &frameFeatures);

    if (nFeatures > 0)
    {

        // we'll use these points to find the bounding rectangle of the matching features
        CvPoint ptMin, ptMax;
        ptMax.x = 0;            
        ptMax.y = 0;
        ptMin.x = frameCopy->width;
        ptMin.y = frameCopy->height;

        struct kd_node* kd_root = kdtree_build(frameFeatures, nFeatures);
        struct feature** nbrs;
        numFeatureMatches = 0;
        for(int i=0; i<numSampleFeatures; i++)
        {
            struct feature *feat = sampleFeatures + i;
            sampleFeatures[i].fwd_match = NULL;
            int k = kdtree_bbf_knn(kd_root, feat, 2, &nbrs, KDTREE_BBF_MAX_NN_CHKS);
            if( k == 2 )
            {
                double d0 = descr_dist_sq(feat, nbrs[0]);
                double d1 = descr_dist_sq(feat, nbrs[1]);
                if(d0 < d1*NN_SQ_DIST_RATIO_THR)
                {
                    // the feature at ptSample in sample image corresponds to ptFrame in current frame
                    CvPoint ptSample = cvPoint(cvRound(feat->x), cvRound(feat->y) );
                    CvPoint ptFrame = cvPoint(cvRound(nbrs[0]->x), cvRound(nbrs[0]->y));
                    ptMin.x = min(ptMin.x, ptFrame.x);  ptMin.y = min(ptMin.y, ptFrame.y);
                    ptMax.x = max(ptMax.x, ptFrame.x);  ptMax.y = max(ptMax.y, ptFrame.y);

                    // draw feature in filter image
                    cvCircle(featureImage, ptSample, 2, colorSwatch[numFeatureMatches % COLOR_SWATCH_SIZE], 3, 8);

                    // draw feature in frame image
                    cvCircle(frameCopy, ptFrame, 2, colorSwatch[numFeatureMatches % COLOR_SWATCH_SIZE], 4, 8);

                    numFeatureMatches++;
                    sampleFeatures[i].fwd_match = nbrs[0];
                }
            }
            free( nbrs );
        }

        // As a starting point, compute the bounding box of matched features (in case we can't find transform)
        CvRect objRect;
        objRect.x = ptMin.x;
        objRect.y = ptMin.y;
        objRect.width = ptMax.x - ptMin.x;
        objRect.height = ptMax.y - ptMin.y;

        if (numFeatureMatches >= SIFT_MIN_RANSAC_FEATURES)
        {
            // try to use RANSAC algorithm to find transformation
            CvMat* H = ransac_xform(sampleFeatures, numSampleFeatures, FEATURE_FWD_MATCH, lsq_homog, 4, 0.01, homog_xfer_err, 3.0, NULL, NULL );
            if (H != NULL)
            {
                //IplImage* xformed;

                double pts[] = {0,0,sampleWidth,0,sampleWidth,sampleHeight,0,sampleHeight};
                CvMat foundRect = cvMat(1, 4, CV_64FC2, pts);
                cvPerspectiveTransform(&foundRect, &foundRect, H);

                cvLine(frameCopy, cvPoint(pts[0],pts[1]), cvPoint(pts[2],pts[3]), CV_RGB(255,255,255), 3);
                cvLine(frameCopy, cvPoint(pts[2],pts[3]), cvPoint(pts[4],pts[5]), CV_RGB(255,255,255), 3);
                cvLine(frameCopy, cvPoint(pts[4],pts[5]), cvPoint(pts[6],pts[7]), CV_RGB(255,255,255), 3);
                cvLine(frameCopy, cvPoint(pts[6],pts[7]), cvPoint(pts[0],pts[1]), CV_RGB(255,255,255), 3);

                objRect.x = min(min(pts[0],pts[2]),min(pts[4],pts[6]));
                objRect.y = min(min(pts[1],pts[3]),min(pts[5],pts[7]));
                objRect.width = max(max(pts[0],pts[2]),max(pts[4],pts[6])) - objRect.x;
                objRect.height = max(max(pts[1],pts[3]),max(pts[5],pts[7])) - objRect.y;

                cvReleaseMat( &H );
            }
        }

        int minMatches = 1 + threshold*6;
        if (numFeatureMatches > minMatches)
        { // we had enough matches to declare the object detected

            // draw object location guess in mask image
            cvRectangle(newMask, cvPoint(objRect.x, objRect.y),
                        cvPoint(objRect.x+objRect.width, objRect.y+objRect.height),
                        cvScalar(0xFF), CV_FILLED, 8);
        }

        kdtree_release( kd_root );
        free(frameFeatures);
    }

    cvResize(featureImage, filterImage);   

    cvResize(frameCopy, applyImage);

    // copy the final output mask
    cvResize(newMask, guessMask);

    cvReleaseImage(&frameCopy);
    cvReleaseImage(&featureImage);
    cvReleaseImage(&newMask);

    updateStandardOutputData();
    return outputData;
}

void SiftClassifier::updateSiftImage()
{
    IplImage *featureImage = cvCloneImage(sampleCopy);
    draw_features(featureImage, sampleFeatures, numSampleFeatures);
    cvResize(featureImage, filterImage);
    cvReleaseImage(&featureImage);
}

void SiftClassifier::save()
{
    if (!isTrained) return;

    Classifier::save();

    char filename[MAX_PATH];

    // save the feature data
    sprintf(filename,"%s/%s",directoryName,classifierDataFileName);

    export_features(filename, sampleFeatures, numSampleFeatures);

    // save the SIFT source sample image
    strcpy(filename, directoryName);
    strcat(filename, FILE_SIFTIMAGE_NAME);
    cvSaveImage(filename, sampleCopy);
}

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
#include "HaarClassifier.h"

/*
LRESULT HaarClassifierDialog::OnDestroy(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled)
{
	TerminateThread(m_hThread, 0);
	parent->isTrained = false;
	if (parent->nStagesCompleted >= MIN_HAAR_STAGES) {
		parent->cascade = cvLoadHaarClassifierCascade(parent->classifierName, cvSize(HAAR_SAMPLE_X, HAAR_SAMPLE_Y));
		if (parent->cascade != NULL) {
			parent->isTrained = true;
            if (parent->isOnDisk) { // this classifier has been saved so we'll update the files
                parent->save();        
            }
		}
	}
	return 0;
}

void HaarClassifierDialog::Train() {
    cvCreateCascadeClassifier(parent->classifierPathname,  parent->vecFilename, parent->negFilename, 
        parent->nPosSamples, parent->nNegSamples, parent->nStages,
		0, 2, .99, .5, .95, 3, 0, 1, HAAR_SAMPLE_X, HAAR_SAMPLE_Y, 3, 0,
		GetDlgItem(IDC_HAAR_PROGRESS), &(parent->nStagesCompleted));
	::EndDialog(m_hWnd, IDOK);
}
*/

HaarClassifier::HaarClassifier() :
        Classifier()
{
    cascade = NULL;
    nStages = START_HAAR_STAGES;
    storage = cvCreateMemStorage(0);
    nPosSamples = 0;
    nNegSamples = 0;
    nStagesCompleted = 0;

    // set the default "friendly name" and type
    strcpy(friendlyName, "Adaboost Recognizer");
    classifierType = ADABOOST_FILTER;        
    // append identifier to directory name
    strcat(directoryName, FILE_HAAR_SUFFIX);
    // default haar classifier data name, override the classifier's name
    sprintf(classifierDataFileName,"%s",FILE_CASCADE_NAME);
}

HaarClassifier::HaarClassifier(const char * pathname) :
        Classifier(pathname)
{
    // set the type
    classifierType = ADABOOST_FILTER;
    cascade = NULL;
    nStages = START_HAAR_STAGES;
    storage = cvCreateMemStorage(0);
    nPosSamples = 0;
    nNegSamples = 0;
    nStagesCompleted = 0;

    char filename[MAX_PATH];
    sprintf(filename,"%s/%s",directoryName,classifierDataFileName);
    // load the cascade from the data file
    load(filename);
}

void HaarClassifier::load(const char * fileName)
{
    // load the cascade from the data file
    cascade = cvLoadHaarClassifierCascade(fileName, cvSize(HAAR_SAMPLE_X, HAAR_SAMPLE_Y));
    if (cascade != NULL)
    {
        isTrained = true;
        isOnDisk = true;
    }
}

void HaarClassifier::load(string fileName)
{
    load(fileName.c_str());
}

HaarClassifier::~HaarClassifier()
{
    cvReleaseMemStorage(&storage);
    if (isTrained)
        cvReleaseHaarClassifierCascade(&cascade);
}

bool HaarClassifier::containsSufficientSamples(TrainingSet *sampleSet)
{
    return ((sampleSet->posSampleCount > 3) && (sampleSet->negSampleCount > 3));
}

void HaarClassifier::prepareData(TrainingSet *sampleSet)
{
    int gridSize = (int) ceil(sqrt((double)sampleSet->posSampleCount));
    int gridX = 0;
    int gridY = 0;
    int gridSampleW = FILTERIMAGE_WIDTH / gridSize;
    int gridSampleH = FILTERIMAGE_HEIGHT / gridSize;
    cvZero(filterImage);

    char imageFilename[MAX_PATH];

    sprintf(vecFilename, "%s/possamples.vec", P_tmpdir);
    sprintf(negFilename, "%s/negsamples.dat", P_tmpdir);
    int classifiernum = (int)time(0);
    sprintf(classifierPathname, "%s/classifier%d/", P_tmpdir, classifiernum);
    sprintf(classifierName, "%s/classifier%d", P_tmpdir, classifiernum);

    icvMkDir(vecFilename);
    icvMkDir(negFilename);
    FILE *vec = fopen(vecFilename, "wb");
    FILE *neglist = fopen(negFilename,"w");
    int imgNum=0;

    icvWriteVecHeader(vec, sampleSet->posSampleCount, HAAR_SAMPLE_X, HAAR_SAMPLE_Y);

    // TODO: call into trainingset class to do this instead of accessing samplemap
    for (map<uint, TrainingSample*>::iterator i = sampleSet->sampleMap.begin(); i != sampleSet->sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_POSSAMPLES)
        { // positive sample

            // create a scaled-down, grayscale version of the sample
            IplImage *sampleCopyColor = cvCreateImage(cvSize(HAAR_SAMPLE_X, HAAR_SAMPLE_Y), IPL_DEPTH_8U, 3);
            IplImage *sampleCopyGrayscale = cvCreateImage(cvSize(HAAR_SAMPLE_X, HAAR_SAMPLE_Y), IPL_DEPTH_8U, 1);
            if (sample->fullImageCopy->width >= HAAR_SAMPLE_X && sample->fullImageCopy->height >= HAAR_SAMPLE_Y)
            {
                cvResize(sample->fullImageCopy, sampleCopyColor, CV_INTER_AREA);
            }
            else
            {
                cvResize(sample->fullImageCopy, sampleCopyColor, CV_INTER_LINEAR);
            }
            cvCvtColor(sampleCopyColor, sampleCopyGrayscale, CV_BGR2GRAY);

            // draw the grayscale version into the filter demo image
            cvCvtColor(sampleCopyGrayscale, sampleCopyColor, CV_GRAY2BGR);
            CvMat *filterImageSubRect = cvCreateMat(gridSampleW, gridSampleH, CV_8UC1);
            cvGetSubRect(filterImage, filterImageSubRect, cvRect(gridX*gridSampleW,gridY*gridSampleH,gridSampleW,gridSampleH));
            cvResize(sampleCopyColor, filterImageSubRect);
            cvReleaseMat(&filterImageSubRect);
            cvReleaseImage(&sampleCopyColor);

            // add the grayscale image to vector of positive samples
            icvWriteVecSample(vec, sampleCopyGrayscale);

            gridX++;
            if (gridX >= gridSize)
            {
                gridX = 0;
                gridY++;
            }

        }
        else if (sample->iGroupId == GROUPID_NEGSAMPLES)
        { // negative sample
            sprintf(imageFilename, "%s/neg%d.jpg", P_tmpdir, imgNum);
            CvSize negImageSize = cvSize(sample->fullImageCopy->width, sample->fullImageCopy->height);

            if ((negImageSize.width < 2*HAAR_SAMPLE_X) || (negImageSize.height < 2*HAAR_SAMPLE_Y))
            {
                negImageSize.width = max(negImageSize.width, 2*HAAR_SAMPLE_X);
                negImageSize.height = max(negImageSize.height, 2*HAAR_SAMPLE_Y);

                IplImage *negImageCopy = cvCreateImage(negImageSize, IPL_DEPTH_8U, 3);
                cvResize(sample->fullImageCopy, negImageCopy);
                cvSaveImage(imageFilename, negImageCopy);
                cvReleaseImage(&negImageCopy);
            }
            else
            {
                cvSaveImage(imageFilename, sample->fullImageCopy);
            }

            fprintf(neglist, "neg%d.jpg\n", imgNum);
            imgNum++;
        }
    }
    fclose(vec);
    fclose(neglist);
    nPosSamples = sampleSet->posSampleCount;
    nNegSamples = sampleSet->negSampleCount;
}

void HaarClassifier::startTraining(TrainingSet* sampleSet)
{
	// Make a copy of the set used for training (we'll want to save it later)
        sampleSet->copyTo(&trainSet);
	prepareData(sampleSet);
}

ClassifierOutputData HaarClassifier::classifyFrame(IplImage *frame)
{
    cvZero(guessMask);
    if (!isTrained) return outputData;
    if (!cascade) return outputData;

    IplImage *newMask = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
    cvZero(newMask);

    // Clear the memory storage we used before
    cvClearMemStorage( storage );

    // There can be more than one object in an image, so we create a growable sequence of objects
    // Detect the objects and store them in the sequence
    CvSeq* objects = cvHaarDetectObjects(frame, cascade, storage,
                                         1.1, (int)(1+threshold*4), CV_HAAR_DO_CANNY_PRUNING,
                                         cvSize(HAAR_SAMPLE_X, HAAR_SAMPLE_Y));

    IplImage *frameCopy = cvCreateImage(cvSize(frame->width,frame->height), IPL_DEPTH_8U, 3);
    cvCopy(frame, frameCopy);

    int objNum = 0;
    // Loop over the found objects
    for(int i = 0; i < (objects ? objects->total : 0); i++ )
    {
        CvRect* r = (CvRect*)cvGetSeqElem(objects, i);
        cvRectangle(frameCopy, cvPoint(r->x,r->y), cvPoint(r->x+r->width,r->y+r->height), colorSwatch[objNum], 2, 8);
        objNum = (objNum + 1) % COLOR_SWATCH_SIZE;

        // draw rectangle in mask image
        cvRectangle(newMask, cvPoint(r->x, r->y), cvPoint(r->x+r->width, r->y+r->height), cvScalar(0xFF), CV_FILLED, 8);
    }

    // copy the final output mask
    cvResize(newMask, guessMask);

    cvResize(frameCopy, applyImage);
    cvReleaseImage(&newMask);
    cvReleaseImage(&frameCopy);

    updateStandardOutputData();
    return outputData;
}

void HaarClassifier::save()
{
    if (!isTrained)
        return;

    Classifier::save();

    char filename[MAX_PATH];

    // save the cascade data
    sprintf(filename,"%s/%s",directoryName,classifierDataFileName);
    cvSave(filename, cascade, 0, 0, cvAttrList(0,0));
}

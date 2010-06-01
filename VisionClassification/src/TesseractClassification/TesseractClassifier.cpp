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
#include "TesseractClassifier.h"

TesseractClassifier::TesseractClassifier() :
        Classifier()
{
    // set the "friendly name" and type
    strcpy(friendlyName, "Tesseract Text Recognition");
    classifierType = FILTER_BUILTIN;

    isTrained = false;
    isOnDisk = false;
    api.InitWithLanguage(NULL, NULL,"eng", NULL, false, 0, NULL);
    // Create the custom output variables for this classifier
    outputData.addVariable("Text", "");
}

TesseractClassifier::TesseractClassifier(const char * pathname) :
        Classifier()
{
    // We will never load standard filters from disk, so this should never be called
    assert(false);
}

TesseractClassifier::~TesseractClassifier()
{
    api.End();
}

bool TesseractClassifier::containsSufficientSamples(TrainingSet *sampleSet)
{
    // Standard filters don't need training samples
    assert(false);
    return true;
}

void TesseractClassifier::startTraining(TrainingSet *sampleSet)
{

    // Standard filters don't use training (this should never be called)
    assert(false);
}

ClassifierOutputData TesseractClassifier::classifyFrame(IplImage *frame)
{
    cvZero(guessMask);
    if(!frame)
        return outputData;

    IplImage *frameGray = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
    IplImage *newMask = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
    cvZero(newMask);

    cvCvtColor(frame, frameGray, CV_BGR2GRAY);
    cvThreshold(frameGray, frameGray, 0, 255, CV_THRESH_BINARY+CV_THRESH_OTSU);

    char * output = api.TesseractRect( (unsigned char*)frameGray->imageData,frameGray->depth/8,frameGray->widthStep,0,0,frameGray->width,frameGray->height);
    //ETEXT_DESC* output = api.Recognize_all_Words();
    /*
    char buffer[TESSERACT_MAX_CHARS];
    int len = 0;

    for (int i = 0; i < output->count; i++)
    {
        const char* ch = output;
        for (int b = 0; b < ch->blanks; ++b)
        {
            buffer[len] = ' ';
            len++;
        }
        if (ch->char_code <= 0x7f)
        {
            buffer[len] = ch->char_code;
            len++;
        }
        cvRectangle(newMask, cvPoint(ch->left, ch->top), cvPoint(ch->right, ch->bottom), cvScalar(0xFF), CV_FILLED);
    }
    buffer[len] = '\0';
    */
    outputData.setVariable("Text", output);

    // copy the final output mask
    cvResize(newMask, guessMask);

    updateStandardOutputData();

    cvReleaseImage(&newMask);

    return outputData;
}


void TesseractClassifier::save()
{
    // We can't save built-in classifiers, so this should never be called
    assert(false);
}

void TesseractClassifier::resetRunningState()
{

}

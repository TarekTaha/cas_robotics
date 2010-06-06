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
#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include "ClassifierOutputData.h"
#include "TrainingSample.h"
#include "TrainingSet.h"
#include "constants.h"
#include "precomp.h"

class Classifier
{
public:

    Classifier();
    Classifier(const char * pathname);
    virtual ~Classifier();

    virtual void startTraining(TrainingSet*) = 0;
    virtual bool containsSufficientSamples(TrainingSet*) = 0;
    virtual ClassifierOutputData classifyFrame(IplImage*) = 0;
    virtual void resetRunningState() = 0;

    virtual void save();
    void configure();
    void deleteFromDisk();
    CvSeq* getMaskContours();
    IplImage* getFilterImage();
    IplImage* getApplyImage();
    const char* getName();
    void setName(const char* newName);
    void activateVariable(const char* varName, bool state);
    void updateStandardOutputData();

    ClassifierOutputData outputData;
    bool isTrained;
    bool isOnDisk;
    int classifierType;
    float threshold;

protected:
    //IplImage *filterIplImage, *applyIplImage;
    IplImage *filterImage, *applyImage, *guessMask;
    CvMemStorage *contourStorage;
    vector<CvRect> boundingBoxes;
    TrainingSet trainSet;	// samples last used to train classifier
    char friendlyName[MAX_PATH];
    char directoryName[MAX_PATH];
};

#endif

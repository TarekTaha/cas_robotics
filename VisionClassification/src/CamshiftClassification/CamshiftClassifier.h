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
#ifndef CAMCLASSIFICATION_H
#define CAMCLASSIFICATION_H

#include "precomp.h"
#include "TrainingSample.h"
#include "TrainingSet.h"
#include "Classifier.h"

class CamshiftClassifier : public Classifier
{
public:
    CamshiftClassifier();
    ~CamshiftClassifier();
    void load(const char * fileName);
    void load(string fileName);
    void startTraining(TrainingSet*);
    void classifyFrame(IplImage*, list<CvRect>*);
    bool containsSufficientSamples(TrainingSet*);
    ClassifierOutputData classifyFrame(IplImage*);
    void resetRunningState();
private:
    IplImage *image, *hsv, *hue, *mask, *backproject, *histimg;
    CvHistogram *hist;

    int hdims,nPosSamples,nNegSamples;
    float hranges_arr[2];
    float* hranges;
};

#endif

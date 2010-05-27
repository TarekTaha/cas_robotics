/***************************************************************************
 *   Vision Classification Library                                         *
 *   Copyright (C) 2010 by:                                                *
 *      Tarek Taha, CAS-UTS  <tataha@cas.edu.au>                           *
 *      Dan Maynes-Aminzade  <monzy@stanford.edu>                          *
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
#ifndef GESTURECLASSIFIER_H
#define GESTURECLASSIFIER_H

#include "constants.h"
#include "TrainingSample.h"
#include "TrainingSet.h"
#include "Classifier.h"
#include "precomp.h"

class GestureClassifier : public Classifier
{
public:
    GestureClassifier();
    GestureClassifier(const char * pathname);
    ~GestureClassifier();

    bool containsSufficientSamples(TrainingSet*);
    void startTraining(TrainingSet*);
    ClassifierOutputData classifyFrame(IplImage*);

    /*!
     Run classifier on current entire motion track
     */
    ClassifierOutputData ClassifyTrack(MotionTrack mt);
    void save();
    /*!
        This classifier doesn't have store any new state info while running live
      */
    void resetRunningState() {}
    void setGraphics(Graphics *);
private:
    void UpdateTrajectoryImage();
    Recognizer rec;
    Graphics *graphics;
    int nTemplates;
    int maxTemplateLength;
};

#endif

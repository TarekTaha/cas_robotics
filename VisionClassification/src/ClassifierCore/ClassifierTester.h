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
#ifndef CLASSIFIERTEST_H
#define CLASSIFIERTEST_H

#include "precomp.h"
#include "constants.h"
#include "Classifier.h"
#include "MotionClassifier.h"
#include "VideoLoader.h"
#include "GestureClassifier.h"

class ClassifierTester
{
public:
    ClassifierTester();
    ~ClassifierTester();
    void TestClassifierOnVideo(Classifier *c, CVideoLoader *vl, int recognizerMode);
    void runClassifierOnCurrentFrame(Classifier *c, CVideoLoader *vl, int recognizerMode);
private:
    IplImage *quickTestImage;
    IplImage *quickTestIplImage;
};

#endif

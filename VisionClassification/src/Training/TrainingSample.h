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
#ifndef TRAININGSAMPLE_H
#define TRAININGSAMPLE_H

#include "constants.h"
#include "Graphics.h"
#include "precomp.h"

class TrainingSample
{
public:
    IplImage *fullImageCopy, *motionHistory;
    uint id;
    int iGroupId, iOrigId;
    CvRect selectBounds;
    MotionTrack motionTrack;

    TrainingSample(IplImage* srcImage,CvRect bounds, int groupId);
    TrainingSample(IplImage* srcImage, IplImage* motionHist, CvRect bounds, int groupId);
    TrainingSample(IplImage *frame, MotionTrack mt, int groupId);
    TrainingSample(char *filename, int groupId);
    TrainingSample(TrainingSample *toClone);
    ~TrainingSample(void);
    void draw(Graphics*);
    void save(char *directory, int index);
    void setGraphics(Graphics *);
private:
    IplImage *resizedImage;
    IplImage *bmpImage;
    int width, height;
    Graphics *graphics;
};

#endif

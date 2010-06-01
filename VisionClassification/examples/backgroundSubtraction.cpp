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
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include "BackgroundSubtraction.h"

int main()
{

    IplImage* frame = NULL,*guessMask=NULL;
    CvCapture* cap = NULL;

    cap   = cvCreateCameraCapture(0);
    frame = cvQueryFrame(cap);
    if(!frame)
    {
        printf("Cant find a camera \n");
        exit(0);
    }
    BackgroundSubtraction backgroundSubtraction;
    //create windows to show background and foreground images
    cvNamedWindow("Background", 1);
    cvNamedWindow("Foreground", 1);
    for(;;)
    {
        frame = cvQueryFrame(cap);
        ClassifierOutputData outputData= backgroundSubtraction.classifyFrame(frame);
        cvShowImage("Foreground", frame);

        if (outputData.hasVariable("Mask"))
        {
            guessMask = outputData.getImageData("Mask");
        }
        cvShowImage("Background", guessMask);
        int k = cvWaitKey(2);
        if( k == 27 )
            break;
    }
    //release capture
    cvReleaseCapture(&cap);
    return 0;

}

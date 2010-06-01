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
#include "iostream"
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include "TesseractClassifier.h"
#include "TrainingSet.h"

int main()
{
    TesseractClassifier tesseractClassifier;
    cvNamedWindow("Detected Text", 1);
    cvNamedWindow("Text Image", 1);
    CvFont font;

    cvInitFont(&font,CV_FONT_HERSHEY_COMPLEX,0.7,0.7,0,1);

    IplImage * textFrame = cvLoadImage("helloworld.bmp",CV_LOAD_IMAGE_GRAYSCALE);
    IplImage * gray      = cvCreateImage(cvGetSize(textFrame),8,3);
    IplImage * displayImg= cvCreateImage(cvGetSize(textFrame),8,3);
    cvZero(displayImg);

    cvCvtColor(textFrame,gray,CV_GRAY2BGR);

    string detectedText;


    ClassifierOutputData outputData= tesseractClassifier.classifyFrame(gray);
    cvShowImage("Text Image", textFrame);

    if (outputData.hasVariable("Text"))
    {
        detectedText = outputData.getStringData("Text");
        cout<<"Text detected:"<<detectedText;
        cvPutText(displayImg,detectedText.c_str(),cvPoint(20,20),&font,cvScalar(0,255,255));
        cvShowImage("Detected Text", displayImg);
    }
    // Display the result for 5 seconds then quit
    cvWaitKey(5000);
    cvReleaseImage(&textFrame);
    cvReleaseImage(&gray);
    cvReleaseImage(&displayImg);
    return 0;
}

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
#include "TrainingSample.h"

// Constructor for a standard training sample (no motion or range data)
TrainingSample::TrainingSample(IplImage* srcImage,CvRect bounds, int groupId):
        graphics(NULL)
{
    // this constructor should only be called for positive and negative sample types
    assert((groupId == GROUPID_POSSAMPLES) || (groupId == GROUPID_NEGSAMPLES));

    iGroupId = groupId;
    iOrigId = groupId;
    selectBounds = bounds;
    motionTrack.clear();
    motionHistory = NULL;

    fullImageCopy = cvCreateImage(cvSize(bounds.width,bounds.height),IPL_DEPTH_8U, 3);
    resizedImage  = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X,LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3);
    bmpImage = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X, LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3);

    cvSetImageROI(srcImage, cvRect( bounds.x, bounds.y, bounds.width, bounds.height));
    cvCopyImage(srcImage, fullImageCopy);

    if (srcImage->width >= LISTVIEW_SAMPLE_X && srcImage->height >= LISTVIEW_SAMPLE_Y) {
        cvResize(srcImage, resizedImage, CV_INTER_AREA);
    } else { 
        cvResize(srcImage, resizedImage, CV_INTER_LINEAR);
    }
    cvResetImageROI(srcImage);
    cvCopyImage(resizedImage, bmpImage);
}

// Constructor for motion training sample (includes motion history image)
TrainingSample::TrainingSample(IplImage* srcImage, IplImage* motionHist, CvRect bounds, int groupId):
        graphics(NULL)
{
    // this constructor should only be called for motion sample type
    assert(groupId == GROUPID_MOTIONSAMPLES);

    iGroupId = groupId;
    iOrigId = groupId;
    selectBounds = bounds;
    motionTrack.clear();

    fullImageCopy = cvCreateImage(cvSize(bounds.width,bounds.height),IPL_DEPTH_8U, 3);
    motionHistory = cvCreateImage(cvSize(motionHist->width,motionHist->height),IPL_DEPTH_32F, 1);
    resizedImage = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X,LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3);
    bmpImage = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X, LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3);

    cvSetImageROI(srcImage, cvRect( bounds.x, bounds.y, bounds.width, bounds.height));
    cvCopyImage(srcImage, fullImageCopy);

    if (srcImage->width >= LISTVIEW_SAMPLE_X && srcImage->height >= LISTVIEW_SAMPLE_Y)
    {
        cvResize(srcImage, resizedImage, CV_INTER_AREA);
    } else
    {
        cvResize(srcImage, resizedImage, CV_INTER_LINEAR);
    }
    cvResetImageROI(srcImage);

    // copy entire frame motion history image (motion history analysis doesn't work as well on partial frame)
    cvCopyImage(motionHist, motionHistory);
    cvCopyImage(resizedImage, bmpImage);
}

// Constructor for range sample (includes motion track)
TrainingSample::TrainingSample(IplImage *frame, MotionTrack mt, int groupId):
        graphics(NULL)
{
    // this constructor should only be called for range sample type
    assert(groupId == GROUPID_RANGESAMPLES);

    iGroupId = groupId;
    iOrigId = groupId;
    motionTrack = mt;
    motionHistory = NULL;

    fullImageCopy = cvCreateImage(cvSize(frame->width,frame->height),IPL_DEPTH_8U, 3);
    cvZero(fullImageCopy);
    cvAddWeighted(frame, 0.5, fullImageCopy, 0.5, 0.0, fullImageCopy);
    
    // draw the trajectory in the sample image
    Template t("", mt);
    if(graphics)
        graphics->drawTrack(fullImageCopy, t.m_points, CV_RGB(100,255,100), 3, GESTURE_SQUARE_SIZE,10);

    resizedImage = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X,LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3); 
    bmpImage = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X, LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3);

    cvResize(fullImageCopy, resizedImage, CV_INTER_AREA);

    cvCopyImage(resizedImage, bmpImage);

}

// Constructor for loading sample from image file
TrainingSample::TrainingSample(char *filename, int groupId):
        graphics(NULL)
{
    // this constructor should only be called for positive and negative sample types
    assert((groupId == GROUPID_POSSAMPLES) || (groupId == GROUPID_NEGSAMPLES));

    iGroupId = groupId;
    iOrigId = groupId;
    motionTrack.clear();
    motionHistory = NULL;

    fullImageCopy = cvLoadImage(filename, 1);
    if (fullImageCopy == NULL)
    {
        fullImageCopy = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X,LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3);
        cvZero(fullImageCopy);
    }

    resizedImage = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X,LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3); 
    bmpImage = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X, LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3);

    if (fullImageCopy->width >= LISTVIEW_SAMPLE_X && fullImageCopy->height >= LISTVIEW_SAMPLE_Y)
    {
        cvResize(fullImageCopy, resizedImage, CV_INTER_AREA);
    }
    else
    {
        cvResize(fullImageCopy, resizedImage, CV_INTER_LINEAR);
    }

    cvCopyImage(resizedImage, bmpImage);
}

// Constructor for cloning an existing sample
TrainingSample::TrainingSample(TrainingSample *toClone):
        graphics(NULL)
{
    iGroupId = toClone->iGroupId;
    iOrigId = toClone->iOrigId;
    selectBounds = toClone->selectBounds;
    id = toClone->id;
    width = toClone->width;
    height = toClone->height;
    motionTrack = toClone->motionTrack;

    fullImageCopy = cvCloneImage(toClone->fullImageCopy);
    if ((iOrigId == GROUPID_MOTIONSAMPLES) && (toClone->motionHistory != NULL))
    {
        motionHistory = cvCloneImage(toClone->motionHistory);
    }
    else
    {
        motionHistory = NULL;
    }
    resizedImage = cvCloneImage(toClone->resizedImage);
    bmpImage = cvCreateImage(cvSize(LISTVIEW_SAMPLE_X, LISTVIEW_SAMPLE_Y),IPL_DEPTH_8U, 3);

    cvCopyImage(resizedImage, bmpImage);
}

TrainingSample::~TrainingSample(void)
{
    cvReleaseImage(&fullImageCopy);
    cvReleaseImage(&resizedImage);
    if (motionHistory != NULL)
        cvReleaseImage(&motionHistory);
    cvReleaseImage(&bmpImage);
}

void TrainingSample::save(char *directory, int index)
{
    char filename[MAX_PATH];

    if (iGroupId == GROUPID_POSSAMPLES) { // positive sample
        sprintf(filename, "%s%s%d%s", directory, FILE_POSIMAGE_PREFIX, index, FILE_IMAGE_EXT);
        cvSaveImage(filename, fullImageCopy);
    } else if (iGroupId == GROUPID_NEGSAMPLES) { // negative sample
        sprintf(filename, "%s%s%d%s", directory, FILE_NEGIMAGE_PREFIX, index, FILE_IMAGE_EXT);
        cvSaveImage(filename, fullImageCopy);
    } else if (iGroupId == GROUPID_MOTIONSAMPLES) { // motion sample
        sprintf(filename, "%s%s%d%s", directory, FILE_MOTIMAGE_PREFIX, index, FILE_IMAGE_EXT);
        cvSaveImage(filename, fullImageCopy);
        sprintf(filename, "%s%s%d%s%s", directory, FILE_MOTIMAGE_PREFIX, index, FILE_IMAGE_EXT, FILE_MOTIONIMAGE_EXT);
        SaveTrackToFile(motionTrack, filename);
    } else if (iGroupId == GROUPID_RANGESAMPLES) { // range sample
        sprintf(filename, "%s%s%d%s", directory, FILE_RNGIMAGE_PREFIX, index, FILE_IMAGE_EXT);
        cvSaveImage(filename, fullImageCopy);
        sprintf(filename, "%s%s%d%s%s", directory, FILE_RNGIMAGE_PREFIX, index, FILE_IMAGE_EXT, FILE_MOTIONTRACK_EXT);
        SaveTrackToFile(motionTrack, filename);
    }
}

void TrainingSample::setGraphics(Graphics *_graphics)
{
    this->graphics = _graphics;
}

void TrainingSample::draw(Graphics *g)
{
    g->drawImage(bmpImage);
}

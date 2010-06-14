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
#include "Classifier.h"

Classifier::Classifier()
{

    isTrained = false;
    isOnDisk = false;
    classifierType = 0;
    threshold = 0.5;
    filterImage = cvCreateImage(cvSize(FILTERIMAGE_WIDTH, FILTERIMAGE_HEIGHT), IPL_DEPTH_8U, 3);
    applyImage  = cvCreateImage(cvSize(FILTERIMAGE_WIDTH, FILTERIMAGE_HEIGHT), IPL_DEPTH_8U, 3);
    guessMask   = cvCreateImage(cvSize(GUESSMASK_WIDTH, GUESSMASK_HEIGHT), IPL_DEPTH_8U, 1);

    // set the standard "friendly name"
    strcpy(friendlyName,"Generic Classifier");

    // create a directory to store this in
    boost::filesystem::path full_path(boost::filesystem::current_path());
    int classifiernum = (int)time(0);
    string rootPath=full_path.string();
    sprintf(directoryName, "%s/%s/%s%d", rootPath.c_str(), APP_CLASS, FILE_CLASSIFIER_PREFIX, classifiernum);

    sprintf(classifierDataFileName,"%s",FILE_DATA_NAME);

    // Initialize contour storage
    contourStorage = cvCreateMemStorage(0);

    // Create the default variables (all classifiers have these)
    outputData.addVariable("Mask", guessMask);
    outputData.addVariable("BoundingBoxes", &boundingBoxes, true);
    outputData.addVariable("NumRegions", (int)0, false);
    outputData.addVariable("TotalArea", (int)0, false);
    CvPoint pt = cvPoint(0,0);
    outputData.addVariable("Centroid", pt, false);
    outputData.addVariable("Contours", (CvSeq*)NULL, false);
}

Classifier::Classifier(const char* pathname)
{
    isTrained = true;
    isOnDisk = true;
    classifierType = 0;
    threshold = 0.5;

    filterImage = cvCreateImage(cvSize(FILTERIMAGE_WIDTH, FILTERIMAGE_HEIGHT), IPL_DEPTH_8U, 3);
    applyImage  = cvCreateImage(cvSize(FILTERIMAGE_WIDTH, FILTERIMAGE_HEIGHT), IPL_DEPTH_8U, 3);
    guessMask   = cvCreateImage(cvSize(GUESSMASK_WIDTH, GUESSMASK_HEIGHT), IPL_DEPTH_8U, 1);

    boost::filesystem::path full_path(pathname);
    if(boost::filesystem::is_directory(full_path))
    {
        strcpy(directoryName,(full_path.string()).c_str());
        sprintf(classifierDataFileName,"%s/%s",(full_path.string()).c_str(),FILE_DATA_NAME);
    }
    else
    {
        strcpy(directoryName,(full_path.parent_path().string()).c_str());
        strcpy(classifierDataFileName,(full_path.filename()).c_str());
    }
    std::cout<<"\nDirname:"<<directoryName<<" classifier datafile name:"<<classifierDataFileName;fflush(stdout);

    // load the "friendly name"
    char filename[MAX_PATH];
    strcpy(filename, directoryName);
    strcat(filename, FILE_FRIENDLY_NAME);
    FILE *namefile = fopen(filename, "r");
    fgets(friendlyName, MAX_PATH, namefile);
    fclose(namefile);

    // load the threshold
    strcpy(filename, directoryName);
    strcat(filename, FILE_THRESHOLD_NAME);
    FILE *threshfile = fopen(filename, "r");
    fread(&threshold, sizeof(float), 1, threshfile);
    fclose(threshfile);

    // load the filter sample image
    strcpy(filename, directoryName);
    strcat(filename, FILE_DEMOIMAGE_NAME);
    IplImage *filterImageCopy = cvLoadImage(filename);
    cvCopy(filterImageCopy, filterImage);
    cvReleaseImage(&filterImageCopy);

    // Initialize contour storage
    contourStorage = cvCreateMemStorage(0);

    // Create the default variables (all classifiers have these)
    outputData.addVariable("Mask", guessMask);
    outputData.addVariable("BoundingBoxes", &boundingBoxes, true);
    outputData.addVariable("NumRegions", (int)0, false);
    outputData.addVariable("TotalArea", (int)0, false);
    CvPoint pt = cvPoint(0,0);
    outputData.addVariable("Centroid", pt, false);
    outputData.addVariable("Contours", (CvSeq*)NULL, false);
}

Classifier::~Classifier()
{
    cvReleaseImage(&filterImage);
    cvReleaseImage(&applyImage);
    cvReleaseImage(&guessMask);
    cvReleaseMemStorage(&contourStorage);
}


void Classifier::save()
{
    char filename[MAX_PATH];
    // make sure the directory exists
    boost::filesystem::create_directory(directoryName);

    // save the "friendly name"
    strcpy(filename,directoryName);
    strcat(filename, FILE_FRIENDLY_NAME);
    FILE *namefile = fopen(filename, "w");
    fputs(friendlyName, namefile);
    fclose(namefile);

    // save the threshold
    strcpy(filename,directoryName);
    strcat(filename, FILE_THRESHOLD_NAME);
    FILE *threshfile  = fopen(filename, "w");
    fwrite(&threshold, sizeof(float), 1, threshfile);
    fclose(threshfile);

    // save the positive and negative training examples along with the classifier
    trainSet.save(directoryName);
    // save the filter sample image
    strcpy(filename, directoryName);
    strcat(filename, FILE_DEMOIMAGE_NAME);
    cvSaveImage(filename, filterImage);
    isOnDisk = true;
}

void Classifier::setClassifierDataFileName(const char* datafileName)
{
    strcpy(classifierDataFileName, datafileName);
}

void Classifier::setDataRootDirName(const char* pathname)
{
    strcpy(directoryName, pathname);
}

void Classifier::configure()
{
}

void Classifier::deleteFromDisk()
{
    if (!isOnDisk)
        return;
    DeleteDirectory(directoryName, true);
    isOnDisk = false;
}

CvSeq* Classifier::getMaskContours()
{
    // reset the contour storage
    cvClearMemStorage(contourStorage);

    CvSeq* contours = NULL;
    cvFindContours(guessMask, contourStorage, &contours, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
    if (contours != NULL)
    {
        contours = cvApproxPoly(contours, sizeof(CvContour), contourStorage, CV_POLY_APPROX_DP, 3, 1 );
    }
    return contours;
}

IplImage* Classifier::getFilterImage()
{
    return filterImage;
}

IplImage* Classifier::getApplyImage()
{
    return applyImage;
}

const char * Classifier::getClassifierDataFileName()
{
    return classifierDataFileName;
}

const char * Classifier::getName()
{
    return friendlyName;
}

void Classifier::setName(const char* newName)
{
    strcpy(friendlyName, newName);
}

void Classifier::activateVariable(const char* varName, bool state)
{
        std::string name = varName;
        outputData.setVariableState(name, state);
}

void Classifier::updateStandardOutputData()
{
    outputData.setVariable("Mask", guessMask);
    CvSeq *contours = getMaskContours();
    outputData.setVariable("Contours", contours);

    // compute bounding boxes of mask contours, along with area and centroid, and count # of regions
    boundingBoxes.clear();
    CvPoint centroid = cvPoint(0,0);
    int nRegions = 0;
    int totalArea = 0;
    if (contours != NULL)
    {
        for (CvSeq *contour = contours; contour != NULL; contour = contour->h_next)
        {
            CvRect cvr = cvBoundingRect(contour, 1);
            totalArea += fabs(cvContourArea(contour));
            CvRect r = cvRect(cvr.x, cvr.y, cvr.width, cvr.height);
            boundingBoxes.push_back(r);
            centroid.x += (cvr.x+cvr.width/2);
            centroid.y += (cvr.y+cvr.height/2);
            nRegions++;
        }
        if (nRegions > 0)
        {
            centroid.x /= nRegions;
            centroid.y /= nRegions;
        }
    }
    outputData.setVariable("BoundingBoxes", &boundingBoxes);
    outputData.setVariable("NumRegions", nRegions);
    outputData.setVariable("TotalArea", totalArea);
    outputData.setVariable("Centroid", centroid);
}

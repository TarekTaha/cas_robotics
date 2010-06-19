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
#include "GestureClassifier.h"

GestureClassifier::GestureClassifier() :
        Classifier(),
        graphics(NULL)
{

    // set the default "friendly name" and type
    strcpy(friendlyName, "Gesture Recognizer");
    classifierType = GESTURE_FILTER;

    // append identifier to directory name
    strcat(directoryName, FILE_GESTURE_SUFFIX);

    // Create the custom output variables for this classifier
    outputData.addVariable("IsMatch", (int)0);
    outputData.addVariable("Gesture", (int)0);
}

GestureClassifier::GestureClassifier(const char * pathname) :
        Classifier(pathname),
        graphics(NULL)
{
    // set the type
    classifierType = GESTURE_FILTER;
    char filename[MAX_PATH];
    sprintf(filename,"%s/%s",directoryName,classifierDataFileName);
    //Load previous data
    load(filename);
    // Create the custom output variables for this classifier
    outputData.addVariable("IsMatch", (int)0);
    outputData.addVariable("Gesture", (int)0);

    updateTrajectoryImage();
}

void GestureClassifier::load(const char * fileName)
{
    // load the templates from the data file
    FILE *datafile = fopen( (fileName), "rb");
    fread(&nTemplates, sizeof(int), 1, datafile);

    for(int i = 0; i < nTemplates; i++)
    {
        Template t(datafile);
        maxTemplateLength = max(maxTemplateLength,t.GetLength());
        char tname[MAX_PATH];
        sprintf(tname, "Gesture %d", i);
        rec.addTemplate(tname, t);
    }
    fclose(datafile);
}

void GestureClassifier::load(string fileName)
{
    load(fileName.c_str());
}

GestureClassifier::~GestureClassifier()
{

}

void GestureClassifier::startTraining(TrainingSet *sampleSet)
{
    // just to filter empty sets (and YES it happens)
    if(sampleSet->posSampleCount<1)
        return;

    // Make a copy of the set used for training (we'll want to save it later)
    sampleSet->copyTo(&trainSet);

    if (isTrained)
    { // delete the old models
        rec.deleteUserTemplates();
    }
    maxTemplateLength = 0;

    nTemplates = sampleSet->rangeSampleCount;

    // TODO: call into trainingset class to do this instead of accessing samplemap
    int tNum = 0;
    for (map<uint, TrainingSample*>::iterator i = sampleSet->sampleMap.begin(); i != sampleSet->sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_RANGESAMPLES)
        {
            // gesture (range) sample
            char tname[MAX_PATH];
            sprintf(tname, "Gesture %d", tNum);
            rec.addTemplate(tname, sample->motionTrack);
            maxTemplateLength = std::max(maxTemplateLength, int(sample->motionTrack.size()));
            tNum++;
        }
    }
    if (isOnDisk)
    {
        // this classifier has been saved so we'll update the files
        save();        
    }

    // update member variables
    isTrained = true;

    // update demo image
    updateTrajectoryImage();
}

bool GestureClassifier::containsSufficientSamples(TrainingSet *sampleSet)
{
    return (sampleSet->rangeSampleCount > 0);
}

ClassifierOutputData GestureClassifier::classifyFrame(IplImage *frame)
{
    // not implemented: this class uses classifyTrack instead
    assert(false);
    ClassifierOutputData data;
    return data;
}    

ClassifierOutputData GestureClassifier::classifyTrack(MotionTrack mt)
{
    cvZero(guessMask);
    outputData.setVariable("IsMatch", 0);
    if (!isTrained)
        return outputData;
    if (mt.size() < GESTURE_MIN_TRAJECTORY_LENGTH)
        return outputData;

    // don't start all the way at the beginning of the track if it's really long
    //int startFrame = max(0, int(mt.size())-GESTURE_MAX_TRAJECTORY_LENGTH);

    cvZero(applyImage);

    Result r = rec.backRecognize(mt);

    if (r.m_score > threshold)
    {
        outputData.setVariable("IsMatch", 1);

        // fill up the mask image
        cvSet(guessMask, cvScalar(0xFF));

        // add a variable for the detected gesture number
        outputData.setVariable("Gesture", r.m_index);

        // draw the recognized gesture in the apply image
        if(graphics)
            graphics->drawTrack(applyImage, rec.m_templates[r.m_index].m_points, colorSwatch[r.m_index % COLOR_SWATCH_SIZE], 3, GESTURE_SQUARE_SIZE,10);

        // print the name of the recognized gesture
        CvFont font;
        cvInitFont(&font,CV_FONT_HERSHEY_COMPLEX_SMALL, 0.55,0.6,0,1, CV_AA);
        cvPutText (applyImage,r.m_name.c_str(),cvPoint(7,applyImage->height-20), &font, cvScalar(255,255,255));
    }

    updateStandardOutputData();
    return outputData;
}

void GestureClassifier::updateTrajectoryImage()
{
    if (nTemplates < 1) return;

    int gridSize = (int) ceil(sqrt((double)nTemplates));
    int gridX = 0;
    int gridY = 0;
    int gridSampleW = FILTERIMAGE_WIDTH / gridSize;
    int gridSampleH = FILTERIMAGE_HEIGHT / gridSize;
    cvZero(filterImage);


    // font for printing name of the gesture
    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_COMPLEX_SMALL, 0.55,0.6,0,1, CV_AA);
    char gestnum[10];

    IplImage *gestureImg = cvCreateImage(cvSize(gridSampleW, gridSampleH), filterImage->depth, filterImage->nChannels);
    for (int i=0; i<nTemplates; i++)
    {
        cvSetImageROI(filterImage, cvRect(gridX*gridSampleW, gridY*gridSampleH, gridSampleW, gridSampleH));
        cvZero(gestureImg);
        if(graphics)
            graphics->drawTrack(gestureImg, rec.m_templates[i].m_points, colorSwatch[i % COLOR_SWATCH_SIZE], 3, GESTURE_SQUARE_SIZE,10);
        sprintf(gestnum, "%d", i);
        cvPutText(gestureImg, gestnum, cvPoint(7,gridSampleH-10), &font, cvScalar(255,255,255));
        cvCopy(gestureImg, filterImage);
        cvResetImageROI(filterImage);
        gridX++;
        if (gridX >= gridSize)
        {
            gridX = 0;
            gridY++;
        }
    }
    cvReleaseImage(&gestureImg);
}

void GestureClassifier::save()
{
    if (!isTrained) return;   
    Classifier::save();
    char filename[MAX_PATH];

    // save the template data
    sprintf(filename,"%s/%s",directoryName,classifierDataFileName);
    FILE *datafile = fopen( (filename), "wb");

    // write out the number of templates
    fwrite(&nTemplates, sizeof(int), 1, datafile);

    // write out all the templates
    for(uint i = 0; i < rec.m_templates.size(); i++)
    {
        rec.m_templates[i].WriteToFile(datafile);
    }
    fclose(datafile);
}

void GestureClassifier::setGraphics(Graphics *_graphics)
{
    this->graphics = _graphics;
}

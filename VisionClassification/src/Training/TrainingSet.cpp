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
#include "TrainingSet.h"

TrainingSet::TrainingSet(void)
{
    posSampleCount = 0;
    negSampleCount = 0;
    motionSampleCount = 0;
    rangeSampleCount = 0;
}

TrainingSet::~TrainingSet(void)
{
    clearSamples();
}

void TrainingSet::getImageList()
{
    return;
}

void TrainingSet::addSample(TrainingSample *sample)
{
    if (sample->iGroupId == GROUPID_POSSAMPLES)
        posSampleCount++;
    else if (sample->iGroupId == GROUPID_NEGSAMPLES)
        negSampleCount++;
    else if (sample->iGroupId == GROUPID_MOTIONSAMPLES)
        motionSampleCount++;
    else if (sample->iGroupId == GROUPID_RANGESAMPLES)
        rangeSampleCount++;
    sampleMap[sample->id] = sample;
}

void  TrainingSet::setSampleGroup(uint sampleId, int groupId)
{
    map<uint, TrainingSample*>::iterator i = sampleMap.find(sampleId);
    if (i != sampleMap.end())
    {
        TrainingSample *sample = i->second;

        // reduce count of old group id
        int oldGroupId = sample->iGroupId;
        if (oldGroupId == GROUPID_POSSAMPLES)
            posSampleCount--;
        else if (oldGroupId == GROUPID_NEGSAMPLES)
            negSampleCount--;
        else if (oldGroupId == GROUPID_MOTIONSAMPLES)
            motionSampleCount--;
        else if (oldGroupId == GROUPID_RANGESAMPLES)
            rangeSampleCount--;

        // increase count of new group id
        if (groupId == GROUPID_POSSAMPLES)
            posSampleCount++;
        else if (groupId == GROUPID_NEGSAMPLES)
            negSampleCount++;
        else if (groupId == GROUPID_MOTIONSAMPLES)
            motionSampleCount++;
        else if (groupId == GROUPID_RANGESAMPLES)
            rangeSampleCount++;

        // set new group id
        sample->iGroupId = groupId;
    }
}

int TrainingSet::getOriginalSampleGroup(uint sampleId)
{
    map<uint, TrainingSample*>::iterator i = sampleMap.find(sampleId);
    if (i != sampleMap.end())
    {
        TrainingSample *sample = i->second;
        return sample->iOrigId;
    }
    return -1;
}

void TrainingSet::removeSample(uint sampleId)
{
    map<uint, TrainingSample*>::iterator i = sampleMap.find(sampleId);
    if (i != sampleMap.end())
    {
        TrainingSample *sample = i->second;
        delete sample;
        sampleMap.erase(i);
    }
}

void TrainingSet::copyTo(TrainingSet *target)
{
    target->clearSamples();
    for (map<uint, TrainingSample*>::iterator i = sampleMap.begin(); i != sampleMap.end(); i++)
    {
        TrainingSample *sample = i->second;
        TrainingSample *sampleCopy = new TrainingSample(sample);
        target->addSample(sampleCopy);
    }
}

void TrainingSet::clearSamples()
{
    for (map<uint, TrainingSample*>::iterator i = sampleMap.begin(); i != sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        delete sample;
    }
    sampleMap.clear();
}

void TrainingSet::save(char *directory)
{
    int posindex = 0, negindex = 0, motindex = 0, rngindex = 0;
    // Write out all the samples
    for (map<uint, TrainingSample*>::iterator i = sampleMap.begin(); i != sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_POSSAMPLES)
        { // positive sample
            sample->save(directory, posindex++);
        }
        else if (sample->iGroupId == GROUPID_NEGSAMPLES)
        { // negative sample
            sample->save(directory, negindex++);
        }
        else if (sample->iGroupId == GROUPID_MOTIONSAMPLES)
        { // negative sample
            sample->save(directory, motindex++);
        }
        else if (sample->iGroupId == GROUPID_RANGESAMPLES)
        { // range sample
            sample->save(directory, rngindex++);
        }
    }
}

void TrainingSet::showSamples()
{
    cvNamedWindow("Samples");

    CvFont font;
    cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 0.7, 0.7, 0, 1, 8);

    // draw the positive samples
    for (map<uint, TrainingSample*>::iterator i = sampleMap.begin(); i != sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_POSSAMPLES)
        {
            IplImage *copy = cvCloneImage(sample->fullImageCopy);
            cvPutText(copy, "Positive Sample", cvPoint(10,10), &font, CV_RGB(0,255,0));
            cvShowImage("Samples", copy);
            cvReleaseImage(&copy);
            cvWaitKey(0);
        }
    }

    // draw the negative samples
    for (map<uint, TrainingSample*>::iterator i = sampleMap.begin(); i != sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_NEGSAMPLES)
        {
            IplImage *copy = cvCloneImage(sample->fullImageCopy);
            cvPutText(copy, "Negative Sample", cvPoint(10,10), &font, CV_RGB(255,0,0));
            cvShowImage("Samples", copy);
            cvReleaseImage(&copy);
            cvWaitKey(0);
        }
    }

    // draw the trash
    for (map<uint, TrainingSample*>::iterator i = sampleMap.begin(); i != sampleMap.end(); i++)
    {
        TrainingSample *sample = (*i).second;
        if (sample->iGroupId == GROUPID_TRASH)
        {
            IplImage *copy = cvCloneImage(sample->fullImageCopy);
            cvPutText(copy, "Trash", cvPoint(10,10), &font, CV_RGB(150,150,150));
            cvShowImage("Samples", copy);
            cvReleaseImage(&copy);
            cvWaitKey(0);
        }
    }

    cvDestroyWindow("Samples");
}

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
#pragma once
#include "SimpleFlowTracker.h"

class CFilterComposer;

class CVideoRunner
{
public: 
	CVideoRunner(CWindow *caller);
	~CVideoRunner();

    static DWORD WINAPI ThreadCallback(CVideoRunner*);
    void StartProcessing(bool isLive);
    void StopProcessing();
	void ApplyFilterChain();
	void ProcessFrame();
	ClassifierOutputData GetStandardOutputData();

    bool AddActiveFilter(Classifier*);
    void ClearActiveFilters();
	void ResetActiveFilterRunningStates();

    bool AddActiveOutput(OutputSink *o);
    void ClearActiveOutputs();

	BOOL LoadRecordedVideo(HWND hwndOwner, CvCapture** capture);

	int videoX, videoY;
    int fps;
    long nFrames, framesAvailable;
    bool processingVideo, runningLive;
    IplImage *copyFrame, *outputFrame, *outputAccImage, *contourMask;
	Bitmap *bmpInput, *bmpOutput, *bmpMotion, *bmpGesture;

    //  keep track of the number of active filters that require motion or blob tracking
    int trackingMotion;
    int trackingGesture;

	// we can combine filters as LIST, AND, OR, or CASCADE
	int filterCombineMode;

	// for the bounding boxes output from a combination of filters
	vector<Rect> boundingBoxes;

private:
    CvCapture *videoCapture;
    IplImage *currentFrame, *guessMask, *combineMask, *combineMaskOutput, *motionHistory;
    IplImage* motionBuf[MOTION_NUM_IMAGES];

    // functions that may be called by ProcessFrame (if motion/gesture filters are active)
    void ProcessMotionFrame();
    void ProcessGestureFrame();

    // for keeping track of position within circular motion history buffer
    int last;

	// for tracking optical flow for gesture tracking
	SimpleFlowTracker m_flowTracker;

    // list of classifiers to apply to live video stream
    list<Classifier*> activeClassifiers;

    // list of outputs to which we will send video data
    list<OutputSink*> activeOutputs;

	// memory storage for contours of combine mask
	CvMemStorage *contourStorage;

	DWORD threadID;
	HANDLE m_hMutex;
	HANDLE m_hThread;
    CWindow *parent;
};

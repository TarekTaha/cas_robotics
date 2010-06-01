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
#include "precomp.h"
#include "constants.h"
#include "TrainingSample.h"
#include "TrainingSet.h"
#include "Classifier.h"
#include "OutputSink.h"
#include "MotionClassifier.h"
#include "GestureClassifier.h"
#include "VideoRunner.h"

CVideoRunner::CVideoRunner(CWindow *caller) {
    videoCapture = NULL;
    copyFrame = NULL;
    outputAccImage = NULL;
	contourMask = NULL;
    motionHistory = NULL;
    bmpInput = NULL;
    bmpOutput = NULL;
	bmpMotion = NULL;
	bmpGesture = NULL;
    processingVideo = false;
	runningLive = true;
    nFrames = 0;
	framesAvailable = 0;
    parent = caller;

    trackingMotion = 0;
    trackingGesture = 0;

	m_hMutex = NULL;
    m_hThread = NULL;
}

CVideoRunner::~CVideoRunner(void) {
    StopProcessing();
	if (m_hMutex) CloseHandle(m_hMutex);
}

ClassifierOutputData CVideoRunner::GetStandardOutputData() {
	ClassifierOutputData outputData;
	cvResize(combineMask, combineMaskOutput);
	outputData.AddVariable("Mask", combineMaskOutput);

	// reset the contour storage
    cvClearMemStorage(contourStorage);
	// find the contours in the combineMask
    CvSeq* contours = NULL;
	cvFindContours(combineMaskOutput, contourStorage, &contours, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
	if (contours != NULL) {
        contours = cvApproxPoly(contours, sizeof(CvContour), contourStorage, CV_POLY_APPROX_DP, 3, 1 );
	}
	outputData.AddVariable("Contours", contours);

	// compute bounding boxes of mask contours, along with area and centroid, and count # of regions
	boundingBoxes.clear();
	Point centroid(0,0);
	int nRegions = 0;
	int totalArea = 0;
	if (contours != NULL){
		for (CvSeq *contour = contours; contour != NULL; contour = contour->h_next) {
			CvRect cvr = cvBoundingRect(contour, 1);
			totalArea += fabs(cvContourArea(contour));
			Rect r(cvr.x, cvr.y, cvr.width, cvr.height);
			boundingBoxes.push_back(r);
			centroid.X += (cvr.x+cvr.width/2);
			centroid.Y += (cvr.y+cvr.height/2);
			nRegions++;
		}
		if (nRegions > 0) {
			centroid.X /= nRegions;
			centroid.Y /= nRegions;
		}
	}
	outputData.AddVariable("BoundingBoxes", &boundingBoxes);
	outputData.AddVariable("NumRegions", nRegions);
	outputData.AddVariable("TotalArea", totalArea);
	outputData.AddVariable("Centroid", centroid);
	return outputData;
}

void CVideoRunner::ApplyFilterChain() {
	USES_CONVERSION;

	bool processedMotion = false, processedGesture = false;
	ClassifierOutputData combinedata;	// used for combining data across multiple classifiers

	int nCurrentFilter = 0;
    int nFiltersInChain = activeClassifiers.size();

	// initialize the "combineMask" appropriately depending upon our combine mode
	if (filterCombineMode == IDC_COMBINE_AND) {
		if (nFiltersInChain > 0) cvSet(combineMask, cvScalar(0xFF));
		else cvZero(combineMask);
	} else if (filterCombineMode == IDC_COMBINE_OR) {
		cvZero(combineMask);
	}

	// step through each active classifier in turn
    for (list<Classifier*>::iterator i=activeClassifiers.begin(); i!=activeClassifiers.end(); i++) {

		// get the output of the classifier and store it in "outdata"
		ClassifierOutputData outdata;

        if ((*i)->classifierType == MOTION_FILTER) {
			if (!processedMotion) {	// this is the first motion filter in the chain, so we'll update the motion image now
				ProcessMotionFrame();
				processedMotion = true;
			}
            outdata = ((MotionClassifier*)(*i))->ClassifyMotion(motionHistory, nFrames);
        } else if ((*i)->classifierType == GESTURE_FILTER) {
			if (!processedGesture) {	// this is the first gesture filter in the chain, so we'll update the motion trails now
				ProcessGestureFrame();
				processedGesture = true;
			}

			MotionTrack mt = m_flowTracker.GetCurrentTrajectory();
            outdata = ((GestureClassifier*)(*i))->ClassifyTrack(mt);
			if (outdata.HasVariable("IsMatch")) {
				if (outdata.GetIntData("IsMatch") != 0) {
					m_flowTracker.ClearCurrentTrajectory();
				}
			}
        } else {
            outdata = (*i)->ClassifyFrame(copyFrame);
        } 

		// pull the mask out of the returned data and store it in "guessMask"
		if (outdata.HasVariable("Mask")) {
			IplImage *mask = outdata.GetImageData("Mask");
			cvResize(mask, guessMask);
		} else {
			cvZero(guessMask);
		}

		if (filterCombineMode == IDC_COMBINE_LIST) {
			// in LIST mode we draw each filter outlined separately in the accumulator frame

			// Copy the masked output of this filter to accumulator frame
			cvZero(outputAccImage);
			cvCopy(copyFrame, outputAccImage, guessMask);

			// Trace contours in accumulator frame
			CvSeq *contours = outdata.GetSequenceData("Contours");
			if (contours != NULL) {
				cvZero(contourMask);
				cvDrawContours(contourMask, contours, cvScalar(0xFF), cvScalar(0x00), 1, 1, CV_AA);
				cvResize(contourMask, guessMask);
				cvSet(outputAccImage, colorSwatch[nCurrentFilter%COLOR_SWATCH_SIZE], guessMask);
			}

			// Add masked accumulator frame to output frame
			cvAddWeighted(outputAccImage, (1.0/nFiltersInChain), outputFrame, 1.0, 0, outputFrame);

			// in LIST mode we apply output chain to each filter output separately
			for (list<OutputSink*>::iterator j=activeOutputs.begin(); j!=activeOutputs.end(); j++) {
				(*j)->ProcessOutput(copyFrame, outdata, W2A((*i)->GetName()));
			}
		} else if (filterCombineMode == IDC_COMBINE_AND){
			// In AND mode we don't draw anything until the end, once we've combined all the outputs.
			// We combine the "guessMask" from each filter into the "combineMask" and draw that.
			cvAnd(guessMask, combineMask, combineMask);
			combinedata.MergeWith(outdata);
		} else if (filterCombineMode == IDC_COMBINE_OR){
			// In OR mode we don't draw anything until the end, once we've combined all the outputs.
			// We combine the "guessMask" from each filter into the "combineMask" and draw that.
			cvOr(guessMask, combineMask, combineMask);
			combinedata.MergeWith(outdata);
		} else if (filterCombineMode == IDC_COMBINE_CASCADE){
			// In CASCADE mode we actually modify the input frame so that the input of the next
			// filter in the chain will include only the regions of the input image that have passed through
			// the earlier filters in the chain.  The final mask is the output of the last filter in the chain.
			cvCopy(copyFrame, outputAccImage);
			cvZero(copyFrame);
			cvCopy(outputAccImage, copyFrame, guessMask);
			cvCopy(guessMask, combineMask);
			combinedata.MergeWith(outdata);
		}
        nCurrentFilter++;
	}

	// If we are in a mode where outputs get combined, we need to output the final result
	// now that we have run all the filters.
	if (filterCombineMode != IDC_COMBINE_LIST) {
		ClassifierOutputData combinemaskdata = GetStandardOutputData();
		combinedata.MergeWith(combinemaskdata);

		cvZero(outputAccImage);
		cvCopy(copyFrame, outputAccImage, combineMask);

		// Trace contours in accumulator frame
		CvSeq *contours = combinedata.GetSequenceData("Contours");
		if (contours != NULL) {
			cvZero(contourMask);
			cvDrawContours(contourMask, contours, cvScalar(0xFF), cvScalar(0x00), 1, 1, CV_AA);
			cvResize(contourMask, guessMask);
			cvSet(outputAccImage, colorSwatch[0], guessMask);
		}

		// copy accumulator frame to output frame
		cvCopy(outputAccImage, outputFrame);

		// now we apply the output chain to the combined output data
		for (list<OutputSink*>::iterator j=activeOutputs.begin(); j!=activeOutputs.end(); j++) {
			(*j)->ProcessOutput(copyFrame, combinedata, "Combination");
		}
	}
}

void CVideoRunner::ProcessFrame() {
    USES_CONVERSION;
	if (currentFrame == NULL) return;

    WaitForSingleObject(m_hMutex,INFINITE);

    // load frame and flip if needed
	if (currentFrame->origin  == IPL_ORIGIN_TL) {
		cvCopy(currentFrame,copyFrame);
	} else {
		cvFlip(currentFrame,copyFrame);
	}

    // some outputs run on the original (unfiltered) frame
	// we apply these before applying any of the filters
    for (list<OutputSink*>::iterator j=activeOutputs.begin(); j!=activeOutputs.end(); j++) {
        (*j)->ProcessInput(copyFrame);
    }

    // First black out the output frame
    cvZero(outputFrame);

    // Now apply filter chain to frame
	ApplyFilterChain();

    // convert to bitmap
    IplToBitmap(copyFrame, bmpInput);
    IplToBitmap(outputFrame, bmpOutput);

    // invalidate parent rectangle for redraw
    CRect videoRect(FILTERLIBRARY_WIDTH, 0, WINDOW_X, WINDOW_Y);
    if (parent->IsWindow()) {
        parent->InvalidateRect(&videoRect, FALSE);
    }

    // update frame count and release the mutex
    nFrames++;
    ReleaseMutex(m_hMutex);

    // Grab next frame (do this AFTER releasing mutex)
	if (runningLive || (nFrames < framesAvailable)) {
		currentFrame = cvQueryFrame(videoCapture);
	} else {	// we are all out of recorded video frames
		parent->SendMessage(WM_COMMAND, IDC_RUNRECORDED, 0);
	}
}

void CVideoRunner::ProcessMotionFrame() {
    // convert frame to grayscale for motion history computation
    cvCvtColor(copyFrame, motionBuf[last], CV_BGR2GRAY);
    int idx1 = last;
    int idx2 = (last + 1) % MOTION_NUM_IMAGES;
    last = idx2;

    // get difference between frames
    IplImage* silh = motionBuf[idx2];
    cvAbsDiff(motionBuf[idx1], motionBuf[idx2], silh);

    // threshold difference image and use it to update motion history image
    cvThreshold(silh, silh, MOTION_DIFF_THRESHOLD, 1, CV_THRESH_BINARY); 
    cvUpdateMotionHistory(silh, motionHistory, nFrames, MOTION_MHI_DURATION); // update MHI

    // convert MHI to blue 8U image
    IplImage *mask = cvCreateImage(cvSize(motionHistory->width, motionHistory->height), IPL_DEPTH_8U, 1);
    IplImage *dst = cvCreateImage(cvSize(motionHistory->width, motionHistory->height), IPL_DEPTH_8U, 3);
    cvCvtScale(motionHistory, mask, 255./MOTION_MHI_DURATION,(MOTION_MHI_DURATION-nFrames)*255./MOTION_MHI_DURATION);
    cvZero(dst);
    cvCvtPlaneToPix( mask, 0, 0, 0, dst );

    IplToBitmap(dst, bmpMotion);
	cvReleaseImage(&mask);
	cvReleaseImage(&dst);
}

void CVideoRunner::ProcessGestureFrame() {
    // Process the new frame with the flow tracker
	m_flowTracker.ProcessFrame(copyFrame);
	IplToBitmap(m_flowTracker.outputFrame, bmpGesture);
}

DWORD WINAPI CVideoRunner::ThreadCallback(CVideoRunner* instance) {
    while (1) {
        if (instance->processingVideo && (instance->currentFrame != NULL)) {
	        instance->ProcessFrame();
        } else {
	        return 1L;
        }
    }
    return 1L;
}

void CVideoRunner::StartProcessing(bool isLive) {
	CvCapture *vc = NULL;
	runningLive = isLive;
	if (isLive) {    // Attempt to access the camera and get dimensions
	    vc = cvCreateCameraCapture(0);
		if (vc == NULL) {
			MessageBox(GetActiveWindow(),
				L"Sorry, I'm unable to connect to a camera.  Please make sure that your camera is plugged in and its drivers are installed.", 
				L"Error Accessing Camera", MB_OK | MB_ICONERROR);
			return;
		}
	} else {	// Attempt to load a recorded video
		if (!LoadRecordedVideo(parent->m_hWnd, &vc)) {
			// user didn't select a file
			return;
		}
		if (vc == NULL) { // user selected a file but we couldn't load it
			MessageBox(GetActiveWindow(), 
				L"Sorry, I'm unable to load this video file.\nIt may be in a format I can't recognize.", 
				L"Error Loading Video", MB_OK | MB_ICONERROR);
			return;
		}
	}

    // get video capture properties
	videoCapture = vc;
    currentFrame = cvQueryFrame(videoCapture);
    videoX = cvGetCaptureProperty(videoCapture, CV_CAP_PROP_FRAME_WIDTH);
    videoY = cvGetCaptureProperty(videoCapture, CV_CAP_PROP_FRAME_HEIGHT);
    fps = cvGetCaptureProperty(videoCapture, CV_CAP_PROP_FPS);
	nFrames = 1;
	if (!runningLive) {
		framesAvailable = cvGetCaptureProperty(videoCapture, CV_CAP_PROP_FRAME_COUNT);
	}

	// create images to store a copy of the current frame input and output, and an accumulator for filter data
    copyFrame = cvCreateImage(cvSize(videoX,videoY),IPL_DEPTH_8U,3);
    outputFrame = cvCreateImage(cvSize(videoX,videoY),IPL_DEPTH_8U,3);
    outputAccImage =  cvCreateImage(cvSize(videoX,videoY),IPL_DEPTH_8U,3);
	contourMask = cvCreateImage(cvSize(GUESSMASK_WIDTH, GUESSMASK_HEIGHT), IPL_DEPTH_8U, 1);
	combineMaskOutput = cvCreateImage(cvSize(GUESSMASK_WIDTH, GUESSMASK_HEIGHT), IPL_DEPTH_8U, 1);

    // create image to store motion history
    motionHistory = cvCreateImage(cvSize(videoX,videoY), IPL_DEPTH_32F, 1);
    cvZero(motionHistory);

    // Allocate an image history ring buffer
    memset(motionBuf, 0, MOTION_NUM_IMAGES*sizeof(IplImage*));
    for(int i = 0; i < MOTION_NUM_IMAGES; i++) {
        motionBuf[i] = cvCreateImage(cvSize(copyFrame->width,copyFrame->height), IPL_DEPTH_8U, 1);
        cvZero(motionBuf[i]);
    }
    last = 0;

    // create a mask to store the results of the processing, and one for combining multiple filters
    guessMask = cvCreateImage(cvSize(videoX,videoY),IPL_DEPTH_8U,1);
    combineMask = cvCreateImage(cvSize(videoX,videoY),IPL_DEPTH_8U,1);

    // Create bitmaps to display video input and output
    bmpInput = new Bitmap(videoX, videoY, PixelFormat24bppRGB);
    bmpOutput = new Bitmap(videoX, videoY, PixelFormat24bppRGB);

	// Create smaller bitmaps for motion and gesture status images
    bmpMotion = new Bitmap(videoX, videoY, PixelFormat24bppRGB);
    bmpGesture = new Bitmap(videoX, videoY, PixelFormat24bppRGB);

	// Initialize some contour storage for tracing combination masks
	contourStorage = cvCreateMemStorage(0);

    processingVideo = true;

    // Start processing thread
	m_hMutex = CreateMutex(NULL,FALSE,NULL);
	m_hThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ThreadCallback, (LPVOID)this, 0, &threadID);
}

void CVideoRunner::StopProcessing() {
    if (!processingVideo) return;

    WaitForSingleObject(m_hMutex,INFINITE);

	// End processing thread
	TerminateThread(m_hThread, 0);
    processingVideo = false;
	currentFrame = NULL;

    cvReleaseCapture(&videoCapture);
    cvReleaseImage(&copyFrame);
    cvReleaseImage(&outputFrame);
    cvReleaseImage(&outputAccImage);
	cvReleaseImage(&contourMask);
    cvReleaseImage(&motionHistory);
    for(int i = 0; i < MOTION_NUM_IMAGES; i++) {
        cvReleaseImage(&motionBuf[i]);
    }
    cvReleaseImage(&guessMask);
    cvReleaseImage(&combineMask);
	cvReleaseImage(&combineMaskOutput);

    delete bmpInput;
    delete bmpOutput;
	delete bmpMotion;
	delete bmpGesture;

	cvReleaseMemStorage(&contourStorage);

    ReleaseMutex(m_hMutex);
}

bool CVideoRunner::AddActiveFilter(Classifier *c) {	// returns true if the filter was added; false if it was already active
	bool alreadyAdded = false;
    WaitForSingleObject(m_hMutex,INFINITE);
    for (list<Classifier*>::iterator j=activeClassifiers.begin(); j!=activeClassifiers.end(); j++) {
		if ((*j) == c) {
			alreadyAdded = true;
		}
    }
	if (!alreadyAdded) {
		if (c->classifierType == MOTION_FILTER) {
			trackingMotion++;
		} else if (c->classifierType == GESTURE_FILTER) {
			trackingGesture++;
		}
		activeClassifiers.push_back(c);
	}
    ReleaseMutex(m_hMutex);
	return !alreadyAdded;
}

void CVideoRunner::ClearActiveFilters() {
    WaitForSingleObject(m_hMutex,INFINITE);
    trackingMotion = 0;
    trackingGesture = 0;
    activeClassifiers.clear();
    ReleaseMutex(m_hMutex);
}

void CVideoRunner::ResetActiveFilterRunningStates() {
    WaitForSingleObject(m_hMutex,INFINITE);
    for (list<Classifier*>::iterator i=activeClassifiers.begin(); i!=activeClassifiers.end(); i++) {
        (*i)->ResetRunningState();
    }
    ReleaseMutex(m_hMutex);
}

bool CVideoRunner::AddActiveOutput(OutputSink *o) {	// returns true if the output was added; false if it was already active
	bool alreadyAdded = false;
    WaitForSingleObject(m_hMutex,INFINITE);
    for (list<OutputSink*>::iterator j=activeOutputs.begin(); j!=activeOutputs.end(); j++) {
		if ((*j) == o) {
			alreadyAdded = true;
		}
    }
	if (!alreadyAdded) {
		activeOutputs.push_back(o);
		o->StartRunning();
	}
    ReleaseMutex(m_hMutex);
	return !alreadyAdded;
}

void CVideoRunner::ClearActiveOutputs() {
    WaitForSingleObject(m_hMutex,INFINITE);
    for (list<OutputSink*>::iterator j=activeOutputs.begin(); j!=activeOutputs.end(); j++) {
        (*j)->StopRunning();
    }
    activeOutputs.clear();
    ReleaseMutex(m_hMutex);
}


BOOL CVideoRunner::LoadRecordedVideo(HWND hwndOwner, CvCapture** capture) {
    WCHAR szFileName[MAX_PATH] = L"";
    OPENFILENAMEW ofn;
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn); // SEE NOTE BELOW
    ofn.hwndOwner = hwndOwner;
    ofn.lpstrFilter = L"Video Files\0*.avi;*.mpg;*.mp4;*.wmv;*.flv;*.mpeg;*.m2v;*.mpv;*.mov;*.qt;*.vob;*.rm\0";
    ofn.lpstrFile = szFileName;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_EXPLORER | OFN_FILEMUSTEXIST | OFN_HIDEREADONLY;
    ofn.lpstrDefExt = L"avi";

    if(!GetOpenFileName(&ofn)) {
	    return FALSE;
    }

    USES_CONVERSION;
    // Attempt to load the video file and get dimensions
    *capture = cvCreateFileCapture(W2A(szFileName));
	return TRUE;
}

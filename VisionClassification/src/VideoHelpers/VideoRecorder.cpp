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
#include "precomp.h"
#include "constants.h"
#include "VideoRecorder.h"

CVideoRecorderDialog::CVideoRecorderDialog(CVideoRecorder *p) :
	videoRect(10, 35, 330, 275), 
	drawRect(10, 35, 320, 240) {
	m_hMutex = NULL;
	parent = p;
}

CVideoRecorderDialog::~CVideoRecorderDialog() {
	if (m_hMutex) CloseHandle(m_hMutex);
}

LRESULT CVideoRecorderDialog::OnInitDialog(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled)
{
    CenterWindow();
	m_hMutex = CreateMutex(NULL,FALSE,NULL);
	m_hThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)ThreadCallback, (LPVOID)this, 0, &threadID);
	return TRUE;    // let the system set the focus
}

LRESULT CVideoRecorderDialog::OnClose(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled) {
    EndDialog(IDCANCEL);
    return 0;
}

LRESULT CVideoRecorderDialog::OnCancel(WORD wNotifyCode, WORD wID, HWND hWndCtl, BOOL& bHandled) {	
	EndDialog(IDCANCEL);
    return 0;
}

LRESULT CVideoRecorderDialog::OnPaint( UINT, WPARAM, LPARAM, BOOL& )
{
	PAINTSTRUCT ps;
    HDC hdc = BeginPaint(&ps);
	Graphics graphics(hdc);
	if (parent->bmpVideo != NULL) {
		graphics.DrawImage(parent->bmpVideo, drawRect);
	}
    EndPaint(&ps);
    return 0;
}

LRESULT CVideoRecorderDialog::OnDestroy(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled) {
	WaitForSingleObject(m_hMutex,INFINITE);
	TerminateThread(m_hThread, 0);
	return 0;
}

DWORD WINAPI CVideoRecorderDialog::ThreadCallback(CVideoRecorderDialog* instance) {
	instance->ConvertFrames();
	return 1L;
}

void CVideoRecorderDialog::ConvertFrames() {
	while (1) {
		WaitForSingleObject(m_hMutex,INFINITE);
		if (parent->currentFrame != NULL) {
			parent->ConvertFrame();

            // Redraw dialog window
			InvalidateRect(&videoRect,FALSE);

            // Release the mutex before trying to grab the next frame
            ReleaseMutex(m_hMutex);
            parent->GrabNextFrame();
		} else {
			ReleaseMutex(m_hMutex);
			EndDialog(0);
			return;
		}
   }
}


CVideoRecorder::CVideoRecorder() :
	m_hVideoRecorderDialog(this) {
    videoCapture = NULL;
    copyFrame = NULL;
    bmpVideo = NULL;
    recordingVideo = false;
    nFrames = 0;
    wcscpy(szFileName, L"");
}

CVideoRecorder::~CVideoRecorder(void) {
    if (recordingVideo) {
        cvReleaseCapture(&videoCapture);
        cvReleaseImage(&copyFrame);
		delete bmpVideo;
    }
}

BOOL CVideoRecorder::RecordVideoFile(HWND hwndOwner) {
    USES_CONVERSION;
    OPENFILENAMEW ofn;
    wcscpy(szFileName, L"");
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = hwndOwner;
    ofn.lpstrFilter = L"Video Files\0*.avi\0";;
    ofn.lpstrCustomFilter = NULL;
    ofn.nFilterIndex = 1;
    ofn.lpstrFile = szFileName;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_EXPLORER | OFN_PATHMUSTEXIST | OFN_HIDEREADONLY | OFN_OVERWRITEPROMPT;
    ofn.lpstrDefExt = L"avi";

    if(!GetSaveFileName(&ofn)) {
		return FALSE;
	}

	// Attempt to access the camera and get dimensions
    CvCapture *vc = cvCreateCameraCapture(0);

    if (vc == NULL) {
		MessageBox(GetActiveWindow(),
			L"Sorry, I'm unable to connect to a camera.  Please make sure that your camera is plugged in and its drivers are installed.", 
			L"Error Accessing Camera", MB_OK | MB_ICONERROR);
		return FALSE;
	}

	videoCapture = vc;

    // TODO: allow user to select camera resolution and other properties
    currentFrame = cvQueryFrame(videoCapture);
    videoX = cvGetCaptureProperty(videoCapture, CV_CAP_PROP_FRAME_WIDTH);
    videoY = cvGetCaptureProperty(videoCapture, CV_CAP_PROP_FRAME_HEIGHT);
    fps = cvGetCaptureProperty(videoCapture, CV_CAP_PROP_FPS);

	// create an image to store a copy of the current frame
    copyFrame = cvCreateImage(cvSize(videoX,videoY),IPL_DEPTH_8U,3);

    // Create a bitmap to display video
    bmpVideo = new Bitmap(videoX, videoY, PixelFormat24bppRGB);

	// create video writer and begin recording
	videoWriter = cvCreateVideoWriter(W2A(szFileName), CV_FOURCC('D','I','V','X'), 30, cvSize(videoX, videoY), 1);

    recordingVideo = true;
	m_hVideoRecorderDialog.DoModal();

    // release camera and video writer
    cvReleaseVideoWriter(&videoWriter);
	cvReleaseCapture(&videoCapture);

	return TRUE;
}

void CVideoRecorder::ConvertFrame() {

	if (currentFrame == NULL) return;

	Rect videoBounds(0,0,videoX,videoY);
	if (currentFrame->origin  == IPL_ORIGIN_TL) {
		cvCopy(currentFrame,copyFrame);
	} else {
		cvFlip(currentFrame,copyFrame);
	}
	cvWriteFrame(videoWriter, copyFrame);
	nFrames++;

    IplToBitmap(copyFrame, bmpVideo);
}

void CVideoRecorder::GrabNextFrame() {
    // Grab next frame
	currentFrame = cvQueryFrame(videoCapture);
}

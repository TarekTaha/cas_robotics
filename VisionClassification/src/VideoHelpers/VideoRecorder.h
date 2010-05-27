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
#pragma once

class CVideoRecorder;

class CVideoRecorderDialog : public CDialogImpl<CVideoRecorderDialog> {
public:
	CVideoRecorderDialog(CVideoRecorder*);
	~CVideoRecorderDialog();

    enum { IDD = IDD_VIDEORECORDER_DIALOG };
    BEGIN_MSG_MAP(CVideoRecorderDialog)
        MESSAGE_HANDLER(WM_PAINT, OnPaint)
        MESSAGE_HANDLER(WM_INITDIALOG, OnInitDialog)
        MESSAGE_HANDLER(WM_CLOSE, OnClose)
        COMMAND_ID_HANDLER(IDCANCEL, OnCancel)
        MESSAGE_HANDLER(WM_DESTROY, OnDestroy)
	END_MSG_MAP()

	LRESULT OnInitDialog(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnPaint(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnClose(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
    LRESULT OnCancel(WORD wNotifyCode, WORD wID, HWND hWndCtl, BOOL& bHandled);
	LRESULT OnDestroy(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	void ConvertFrames();

private:
	CRect videoRect;
	Rect drawRect;
	DWORD threadID;
	HANDLE m_hMutex;
	HANDLE m_hThread;
	static DWORD WINAPI ThreadCallback(CVideoRecorderDialog*);
	CVideoRecorder *parent;
};

class CVideoRecorder
{
public: 
	CVideoRecorder();
	~CVideoRecorder();
	BOOL RecordVideoFile(HWND);
	void ConvertFrame();
    void GrabNextFrame();

    WCHAR szFileName[MAX_PATH];
	int videoX, videoY;
    int fps;
    int nFrames;
    bool recordingVideo;
    IplImage *copyFrame;
    Bitmap *bmpVideo;

private:
    CvCapture *videoCapture;
	CvVideoWriter *videoWriter;
	
    IplImage *currentFrame;

	friend class CVideoRecorderDialog;
	CVideoRecorderDialog m_hVideoRecorderDialog;
};

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
#include "TrainingSample.h"
#include "TrainingSet.h"
#include "VideoLoader.h"
#include "VideoRecorder.h"
#include "BrightnessClassifier.h"
#include "ColorClassifier.h"
#include "ShapeClassifier.h"
#include "SiftClassifier.h"
#include "HaarClassifier.h"
#include "MotionClassifier.h"
#include "GestureClassifier.h"
#include "FilterSelect.h"
#include "VideoControl.h"
#include "ClassifierTester.h"
#include "VideoMarkup.h"

void AddListViewGroup(HWND hwndList, WCHAR *szText, int iGroupId) {
    LVGROUP lvgrp = { sizeof(lvgrp) };
    lvgrp.state = LVGS_NORMAL;
    lvgrp.mask      = LVGF_HEADER | LVGF_GROUPID | LVGF_ALIGN;
    lvgrp.pszHeader = szText;
    lvgrp.cchHeader = (int)wcslen(lvgrp.pszHeader);
    lvgrp.iGroupId  = iGroupId;
    lvgrp.uAlign    = LVGA_HEADER_CENTER;
    ListView_InsertGroup(hwndList, iGroupId, &lvgrp);
}

CVideoMarkup::CVideoMarkup() :
    m_sampleListView(WC_LISTVIEW, this, 1),
    m_filterSelect(this),
    m_videoControl(this),
    m_videoRect(0,0,VIDEO_X,VIDEO_Y),
    m_filterRect(0, VIDEO_Y+SLIDER_Y, VIDEO_X, WINDOW_Y),
    labelFont(L"Verdana", 10),
    bigFont(L"Verdana", 14),
	stringFormat(),
    posSelectPen(Color(100,100,255,100),2),
    negSelectPen(Color(100,255,100,100),2),
    guessPen(Color(100,255,100),4),
    posBrush(Color(50,100,255,100)),
    negBrush(Color(50,255,100,100)),
    hoverBrush(Color(25, 50, 150, 255)),
    grayBrush(Color(150, 0, 0, 0)),
	blackBrush(Color(0,0,0)),
	whiteBrush(Color(255,255,255)),
    ltgrayBrush(Color(240,240,240)) {
	guessPen.SetLineJoin(LineJoinRound);

    // TODO: all non window-related variables should be initialized here instead of in OnCreate
    classifier = new ColorClassifier();
    recognizerMode = COLOR_FILTER;
    showGuesses = false;
    selectingRegion = false;
    draggingIcon = false;
	scrubbingVideo = false;
	currentGroupId = GROUPID_POSSAMPLES;
}


CVideoMarkup::~CVideoMarkup() {
    bool currentClassifierInList = false;
    // free memory used to store classifiers loaded from disk
    for (list<Classifier*>::iterator i = savedClassifiers.begin(); i!= savedClassifiers.end(); i++) {
        if ((*i) == classifier) currentClassifierInList = true;
        delete (*i);
    }
    if (!currentClassifierInList) {
        delete classifier;
    }
}

void CVideoMarkup::EnableControls(BOOL enabled) {
    m_filterSelect.EnableWindow(enabled);
    if ((!m_videoLoader.videoLoaded) || (enabled && !classifier->isTrained)) {
        m_filterSelect.GetDlgItem(IDC_SHOWBUTTON).EnableWindow(FALSE);
        m_filterSelect.GetDlgItem(IDC_QUICKTEST).EnableWindow(FALSE);
        m_filterSelect.GetDlgItem(IDC_SAVEFILTER).EnableWindow(FALSE);
    }
    if (!m_videoLoader.videoLoaded) {
    	m_videoControl.EnableWindow(FALSE);
    } else {
    	m_videoControl.EnableWindow(enabled);
    }
    m_sampleListView.EnableWindow(enabled);
}

LRESULT CVideoMarkup::OnPaint( UINT, WPARAM, LPARAM, BOOL& ) {
	PAINTSTRUCT ps;

    HDC hdc = BeginPaint(&ps);
    Rect drawBounds(0,0,VIDEO_X,VIDEO_Y);
    Rect videoBounds(0,0,m_videoLoader.videoX,m_videoLoader.videoY);
    Rect videoBoundsExt(2,2,m_videoLoader.videoX-4,m_videoLoader.videoY-4);

    if (m_videoLoader.videoLoaded) {
        graphics->SetClip(drawBounds);

        if (m_videoLoader.bmpVideo != NULL) {
            if (showGuesses && !scrubbingVideo) { // highlight computer's guesses
                graphics->DrawImage(m_videoLoader.GetMaskedBitmap(),drawBounds);
            } else {
                graphics->DrawImage(m_videoLoader.bmpVideo,drawBounds);
            }
        }

        Rect selectRect;
        selectRect.X = (INT) min(selectStart.X, selectCurrent.X);
        selectRect.Y = (INT) min(selectStart.Y, selectCurrent.Y);
        selectRect.Width = (INT) abs(selectStart.X - selectCurrent.X);
        selectRect.Height = (INT) abs(selectStart.Y - selectCurrent.Y);

        if (selectingRegion) {
            if (currentGroupId == GROUPID_POSSAMPLES) {
                graphics->FillRectangle(&posBrush, selectRect);
                graphics->DrawRectangle(&posSelectPen, selectRect);
            } else {
                graphics->FillRectangle(&negBrush, selectRect);
                graphics->DrawRectangle(&negSelectPen, selectRect);
            }
        }
        graphics->ResetClip();
    }

    graphicsExamples->FillRectangle(&ltgrayBrush, Rect(0,0,EXAMPLEWINDOW_WIDTH,EXAMPLEWINDOW_HEIGHT));
    if (classifier->isTrained) {
        graphicsExamples->DrawImage(classifier->GetFilterImage(),10,0);
	    graphicsExamples->DrawString(L"RECOGNIZER MODEL", 16, &labelFont, PointF(15,5), &whiteBrush);
	}
    if (showGuesses) {
        graphicsExamples->DrawImage(classifier->GetApplyImage(),FILTERIMAGE_WIDTH+20, 0);
	    graphicsExamples->DrawString(L"RECOGNIZER OUTPUT", 17, &labelFont, PointF(FILTERIMAGE_WIDTH+25,5), &whiteBrush);
    }
	if (classifier->isOnDisk) {
		LPWSTR name = classifier->GetName();
		graphicsExamples->DrawString(L"Currently\nActive:", 17, &labelFont, PointF(2*FILTERIMAGE_WIDTH+30,10), &blackBrush);
	    graphicsExamples->DrawString(name, wcslen(name), &bigFont,
			RectF(2*FILTERIMAGE_WIDTH+30,50,EXAMPLEWINDOW_WIDTH-(2*FILTERIMAGE_WIDTH+30),EXAMPLEWINDOW_HEIGHT-50),
			&stringFormat, &blackBrush);
	}

    if (recognizerMode == GESTURE_FILTER) {
        // draw the current gesture motion trajectory in this frame
        MotionTrack mt = m_videoLoader.GetTrajectoryAtCurrentFrame();
		mt = ScaleToSquare(mt, 3*VIDEO_Y/4);
		mt = TranslateToOrigin(mt);
		DrawTrack(graphics, mt, VIDEO_X, VIDEO_Y, VIDEO_X);
    }

    BitBlt(hdc,0,0,VIDEO_X,VIDEO_Y,hdcmem,0,0,SRCCOPY);
    BitBlt(hdc,EXAMPLEWINDOW_X,EXAMPLEWINDOW_Y,EXAMPLEWINDOW_WIDTH,EXAMPLEWINDOW_HEIGHT,hdcmemExamples,0,0,SRCCOPY);
    EndPaint(&ps);

    return 0;
}

LRESULT CVideoMarkup::OnButtonDown( UINT, WPARAM wParam, LPARAM lParam, BOOL& ) {
    POINT p;
    p.x = LOWORD(lParam);
    p.y = HIWORD(lParam);

    if (!m_videoLoader.videoLoaded) return 0;
    if (!m_videoRect.PtInRect(p)) return 0;

    selectingRegion = true;

    // If the right button is down, we consider this a negative sample
    // TODO: do this some better way
    if (MK_RBUTTON & wParam) currentGroupId = GROUPID_NEGSAMPLES;
    else currentGroupId = GROUPID_POSSAMPLES;

    selectStart.X = (REAL) p.x;
    selectStart.Y = (REAL) p.y;
    selectCurrent = selectStart;

    // restrict mouse movement to within the video window
    CRect videoRect = m_videoRect;
    ClientToScreen(&videoRect);
    ClipCursor(videoRect);

    return 0;
}

LRESULT CVideoMarkup::OnMouseMove( UINT, WPARAM wParam, LPARAM lParam, BOOL& )
{
    POINT p;
    p.x = LOWORD(lParam);
    p.y = HIWORD(lParam);

    if ((wParam & MK_LBUTTON) || (wParam & MK_RBUTTON)) { // mouse is down
        if (draggingIcon) { // we are dragging in listview

            // Determine which group the cursor is over
            LVHITTESTINFO lvhti;
            lvhti.pt = p;
            ClientToScreen(&lvhti.pt);
            ::ScreenToClient(m_sampleListView, &lvhti.pt);
            ListView_HitTestEx(m_sampleListView, &lvhti);
            CRect posRect, negRect, motionRect, rangeRect;
            ListView_GetGroupRect(m_sampleListView, GROUPID_POSSAMPLES, LVGGR_GROUP, &posRect);
            ListView_GetGroupRect(m_sampleListView, GROUPID_NEGSAMPLES, LVGGR_GROUP, &negRect);
            ListView_GetGroupRect(m_sampleListView, GROUPID_MOTIONSAMPLES, LVGGR_GROUP, &motionRect);
            ListView_GetGroupRect(m_sampleListView, GROUPID_RANGESAMPLES, LVGGR_GROUP, &rangeRect);
            if (posRect.PtInRect(lvhti.pt)) { // highlight positive group
                SetCursor(hDropCursor);
                dragHover = true;
                hoverRect.X = posRect.left; hoverRect.Y = posRect.top;
                hoverRect.Width = posRect.Width();  hoverRect.Height = posRect.Height();
            } else if (negRect.PtInRect(lvhti.pt)) { // highlight negative group
                SetCursor(hDropCursor);
                dragHover = true;
                hoverRect.X = negRect.left; hoverRect.Y = negRect.top;
                hoverRect.Width = negRect.Width();  hoverRect.Height = negRect.Height();
            } else if (motionRect.PtInRect(lvhti.pt)) { // highlight motion group
                SetCursor(hDropCursor);
                dragHover = true;
                hoverRect.X = motionRect.left; hoverRect.Y = motionRect.top;
                hoverRect.Width = motionRect.Width();  hoverRect.Height = motionRect.Height();
            } else if (rangeRect.PtInRect(lvhti.pt)) { // highlight gesture group
                SetCursor(hDropCursor);
                dragHover = true;
                hoverRect.X = rangeRect.left; hoverRect.Y = rangeRect.top;
                hoverRect.Width = rangeRect.Width();  hoverRect.Height = rangeRect.Height();
            } else {
                SetCursor(hTrashCursor);
                dragHover = false;
            }

            // update listview to highlight group on hover
            m_sampleListView.RedrawWindow();

            // draw drag icon
            ClientToScreen(&p);
            ImageList_DragMove(p.x, p.y);

         } else if (m_videoLoader.videoLoaded) { // we are selecting a region
            if (!m_videoRect.PtInRect(p)) return 0;

            selectCurrent.X = (REAL) p.x;
            selectCurrent.Y = (REAL) p.y;
           
            InvalidateRgn(activeRgn, FALSE);
        }
    }
	return 0;
}

LRESULT CVideoMarkup::OnButtonUp( UINT, WPARAM, LPARAM lParam, BOOL&)
{
    POINT p;
    p.x = LOWORD(lParam);
    p.y = HIWORD(lParam);

    if (draggingIcon) { // we just completed an icon drag
        // End the drag-and-drop process
        draggingIcon = FALSE;
        ImageList_DragLeave(m_sampleListView);
        ImageList_EndDrag();
        ImageList_Destroy(hDragImageList);
        SetCursor(LoadCursor(NULL, IDC_ARROW));
        ReleaseCapture();

        // Determine the position of the drop point
        LVHITTESTINFO lvhti;
        lvhti.pt = p;
        ClientToScreen(&lvhti.pt);
        ::ScreenToClient(m_sampleListView, &lvhti.pt);
        ListView_HitTestEx(m_sampleListView, &lvhti);
        CRect posRect, negRect, motionRect, rangeRect;
        ListView_GetGroupRect(m_sampleListView, GROUPID_POSSAMPLES, LVGGR_GROUP, &posRect);
        ListView_GetGroupRect(m_sampleListView, GROUPID_NEGSAMPLES, LVGGR_GROUP, &negRect);
        ListView_GetGroupRect(m_sampleListView, GROUPID_MOTIONSAMPLES, LVGGR_GROUP, &motionRect);
        ListView_GetGroupRect(m_sampleListView, GROUPID_RANGESAMPLES, LVGGR_GROUP, &rangeRect);

        int newGroupId;
        if (posRect.PtInRect(lvhti.pt)) newGroupId = GROUPID_POSSAMPLES;
        else if (negRect.PtInRect(lvhti.pt)) newGroupId = GROUPID_NEGSAMPLES;
        else if (motionRect.PtInRect(lvhti.pt)) newGroupId = GROUPID_MOTIONSAMPLES;
        else if (rangeRect.PtInRect(lvhti.pt)) newGroupId = GROUPID_RANGESAMPLES;
        else newGroupId = GROUPID_TRASH;

        // update group membership of selected items based on drop location
        int numSelected = ListView_GetSelectedCount(m_sampleListView);
        int iSelection = -1;
        for (int iIndex=0; iIndex<numSelected; iIndex++) {

            // retrieve the selected item 
            LVITEM lvi;
            iSelection = ListView_GetNextItem(m_sampleListView, iSelection, LVNI_SELECTED);
            lvi.mask = LVIF_IMAGE | LVIF_STATE | LVIF_GROUPID;
            lvi.state = 0;
            lvi.stateMask = 0;
            lvi.iItem = iSelection;
            lvi.iSubItem = 0;
            ListView_GetItem(m_sampleListView, &lvi);

			// Get the ID of this selected item
            UINT sampleId = ListView_MapIndexToID(m_sampleListView, iSelection);

			// test if this is an allowable group membership change
			int origGroupId = sampleSet.GetOriginalSampleGroup(sampleId);
			if (!GroupTransitionIsAllowed(origGroupId, newGroupId)) {
				// this is not a valid change so we'll move to the next item
				continue;
			}

            // update sample group in training set
            sampleSet.SetSampleGroup(sampleId, newGroupId);

            // Update item in list view with new group id
			lvi.iGroupId = newGroupId;
            ListView_SetItem(m_sampleListView, &lvi);
        }
        m_sampleListView.Invalidate(FALSE);

    } else if (m_videoLoader.videoLoaded && selectingRegion) { // we just finished drawing a selection
        ClipCursor(NULL);   // restore full cursor movement
        if (!m_videoRect.PtInRect(p)) {
            InvalidateRect(&m_videoRect,FALSE);
            return 0;
        }
        selectingRegion = false;

        Rect selectRect;
        selectRect.X = (INT) min(selectStart.X, selectCurrent.X);
        selectRect.Y = (INT) min(selectStart.Y, selectCurrent.Y);
        selectRect.Width = (INT) abs(selectStart.X - selectCurrent.X);
        selectRect.Height = (INT) abs(selectStart.Y - selectCurrent.Y);

        Rect drawBounds(0,0,VIDEO_X,VIDEO_Y);
        selectRect.Intersect(drawBounds);
        double scaleX = ((double)m_videoLoader.videoX) / ((double)VIDEO_X);
        double scaleY = ((double)m_videoLoader.videoY) / ((double)VIDEO_Y);
        selectRect.X = (INT) (scaleX * selectRect.X);
        selectRect.Y = (INT) (scaleY * selectRect.Y);
        selectRect.Width = (INT) (scaleX * selectRect.Width);
        selectRect.Height = (INT) (scaleY * selectRect.Height);

        // discard tiny samples since they won't help
        if ((selectRect.Width > 10) && (selectRect.Height > 10)) {
			TrainingSample *sample;
			// if we're in motion mode, the behavior is a little special
			if (recognizerMode == MOTION_FILTER) {
				sample = new TrainingSample(m_videoLoader.copyFrame, m_videoLoader.GetMotionHistory(), m_sampleListView, m_hImageList, selectRect, GROUPID_MOTIONSAMPLES);
			} else {
				sample = new TrainingSample(m_videoLoader.copyFrame, m_sampleListView, m_hImageList, selectRect, currentGroupId);
			}
            sampleSet.AddSample(sample);
        }
        InvalidateRect(&m_videoRect, FALSE);
    }
	return 0;
}

LRESULT CVideoMarkup::OnTrack( UINT, WPARAM wParam, LPARAM, BOOL& ) {
    long sliderPosition =
        (long) ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_GETPOS, 0, 0);
    selectingRegion = false;
	if (LOWORD(wParam) == SB_ENDSCROLL) {
		scrubbingVideo = false;
	} else {
		scrubbingVideo = true;
	}
    selectStart.X = 0;
    selectStart.Y = 0;
    selectCurrent = selectStart;

    m_videoLoader.LoadFrame(sliderPosition);
    if (showGuesses && !scrubbingVideo) {
		RunClassifierOnCurrentFrame();
    }
    InvalidateRgn(activeRgn, FALSE);
    return 0;
}

LRESULT CVideoMarkup::OnCreate(UINT, WPARAM, LPARAM, BOOL& )
{
    // create graphics structures for video and examples
	HDC hdc = GetDC();
	hdcmem = CreateCompatibleDC(hdc);
	hdcmemExamples = CreateCompatibleDC(hdc);
	hbm = CreateCompatibleBitmap(hdc,WINDOW_X,WINDOW_Y);
	hbmExamples = CreateCompatibleBitmap(hdc,EXAMPLEWINDOW_WIDTH,EXAMPLEWINDOW_HEIGHT);
	SelectObject(hdcmem,hbm);
	SelectObject(hdcmemExamples,hbmExamples);
	ReleaseDC(hdc);
    graphics = new Graphics(hdcmem);
    graphicsExamples = new Graphics(hdcmemExamples);
	graphics->SetSmoothingMode(SmoothingModeAntiAlias);
    graphics->Clear(Color(255,255,255));
    graphicsExamples->Clear(Color(255,255,255));

    // create the active window region to invalidate
    HRGN filterRgn = CreateRectRgnIndirect(&m_filterRect);
    HRGN videoRgn = CreateRectRgnIndirect(&m_videoRect);
    activeRgn = CreateRectRgnIndirect(&m_filterRect);
    CombineRgn(activeRgn, filterRgn, videoRgn, RGN_OR);
    DeleteObject(filterRgn);
    DeleteObject(videoRgn);

    hTrashCursor = LoadCursor(_AtlBaseModule.GetResourceInstance(), MAKEINTRESOURCE(IDC_TRASHCURSOR));
    hDropCursor = LoadCursor(_AtlBaseModule.GetResourceInstance(), MAKEINTRESOURCE(IDC_DROPCURSOR));

    // Create the list of samples
    m_sampleListView.Create(m_hWnd, CRect(VIDEO_X,0,WINDOW_X-5,VIDEO_Y-50), _T(""), WS_CHILD | WS_VISIBLE | WS_DISABLED | WS_BORDER | LVS_ICON | LVS_AUTOARRANGE);

    // create the list of example images
    m_hImageList = ImageList_Create(LISTVIEW_SAMPLE_X, LISTVIEW_SAMPLE_Y, ILC_COLOR32 | ILC_MASK, 0, MAX_SAMPLES);
    ListView_SetImageList(m_sampleListView, m_hImageList, LVSIL_NORMAL);
//    ListView_SetIconSpacing(m_sampleListView, LISTVIEW_SAMPLE_X*2, LISTVIEW_SAMPLE_Y*1.5);
    ListView_SetIconSpacing(m_sampleListView, LISTVIEW_SAMPLE_X+20, LISTVIEW_SAMPLE_Y+20);
    ListView_EnableGroupView(m_sampleListView, TRUE);
    ListView_SetExtendedListViewStyle(m_sampleListView, LVS_EX_DOUBLEBUFFER | LVS_EX_BORDERSELECT | LVS_EX_FLATSB);

    // create an image list for the group header images
    //hGroupHeaderImages = ImageList_Create(400,50,ILC_COLOR32 | ILC_MASK,2,2);
    //HBITMAP hbmPos = LoadBitmap(_Module.get_m_hInst(), MAKEINTRESOURCE(IDB_THUMBSUP));
    //HBITMAP hbmNeg = LoadBitmap(_Module.get_m_hInst(), MAKEINTRESOURCE(IDB_THUMBSDOWN));
    //ImageList_Add(hGroupHeaderImages, hbmPos, NULL);
    //ImageList_Add(hGroupHeaderImages, hbmNeg, NULL);
    //DeleteObject(hbmPos);
    //DeleteObject(hbmNeg);
    //ListView_SetGroupHeaderImageList(m_sampleListView, hGroupHeaderImages);

    // add the "positive" and "negative" groups
    AddListViewGroup(m_sampleListView, L"Positive Image Examples", GROUPID_POSSAMPLES);
    AddListViewGroup(m_sampleListView, L"Negative Image Examples", GROUPID_NEGSAMPLES);
    AddListViewGroup(m_sampleListView, L"Motion Examples", GROUPID_MOTIONSAMPLES);
    AddListViewGroup(m_sampleListView, L"Video Range Examples", GROUPID_RANGESAMPLES);
    AddListViewGroup(m_sampleListView, L"Trash", GROUPID_TRASH);

    // Create the video slider
    m_videoControl.Create(m_hWnd, WS_CHILD | WS_VISIBLE | WS_DISABLED );
    m_videoControl.MoveWindow(0,VIDEO_Y,VIDEO_X,SLIDER_Y);
    m_videoControl.ShowWindow(TRUE);
    m_videoControl.EnableWindow(FALSE);
    m_videoControl.EnableSelectionRange(false);
	
    // Create the filter selector
    m_filterSelect.Create(m_hWnd, WS_CHILD | WS_VISIBLE | WS_DISABLED);
    m_filterSelect.MoveWindow(VIDEO_X, VIDEO_Y-50, WINDOW_X-VIDEO_X, WINDOW_Y-VIDEO_Y+50);

	// Start with "Color" mode selected
	m_filterSelect.SelectFilter(COLOR_FILTER);

	m_filterSelect.ShowWindow(TRUE);
    m_filterSelect.EnableWindow(FALSE);

    return 0;
}


LRESULT CVideoMarkup::OnDestroy( UINT, WPARAM, LPARAM, BOOL& ) {
    ImageList_RemoveAll(m_hImageList);
    ImageList_Destroy(m_hImageList);
	delete graphics;
    delete graphicsExamples;
	DeleteDC(hdcmem);
    DeleteDC(hdcmemExamples);
	DeleteObject(hbm);
    DeleteObject(hbmExamples);
    DeleteObject(hTrashCursor);
    DeleteObject(hDropCursor);
    DeleteObject(activeRgn);
    m_sampleListView.DestroyWindow();
    m_filterSelect.DestroyWindow();
    m_videoControl.DestroyWindow();
    // TODO: non window-related variables should be deleted in destructor instead of here
    PostQuitMessage( 0 );
	return 0;
}

LRESULT CVideoMarkup::OnLoadFilter( UINT, WPARAM wParam, LPARAM lParam, BOOL& bHandled) {

	bool isAlreadyLoaded = (classifier == (Classifier*)lParam);

	m_filterSelect.SelectFilter(wParam);
	recognizerMode = wParam;
    switch(wParam) {
        case COLOR_FILTER:
            ReplaceClassifier((ColorClassifier*)lParam);
            break;
        case SHAPE_FILTER:
            ReplaceClassifier((ShapeClassifier*)lParam);
            break;
        case SIFT_FILTER:
            ReplaceClassifier((SiftClassifier*)lParam);
            break;
        case BRIGHTNESS_FILTER:
            ReplaceClassifier((BrightnessClassifier*)lParam);
            break;
        case ADABOOST_FILTER:
            ReplaceClassifier((HaarClassifier*)lParam);
            break;
        case MOTION_FILTER:
            ReplaceClassifier((MotionClassifier*)lParam);
            break;
        case GESTURE_FILTER:
            ReplaceClassifier((GestureClassifier*)lParam);
            break;
    }
    InvalidateRgn(activeRgn, FALSE);
    return isAlreadyLoaded;
}

LRESULT CVideoMarkup::OnCommand( UINT, WPARAM wParam, LPARAM lParam, BOOL& bHandled) {
    USES_CONVERSION;
	bool needToRerunClassifier = false;
    long sliderPosition, sliderRange, selStart, selEnd;
    switch(LOWORD(wParam)) { // this tells us which control the message came from
        case IDC_TRAINBUTTON:
            {
                WCHAR errorMessage[1000] = L"Sorry, you don't have enough examples to train this recognizer. Please add some more examples and try again.\n";
                if (!classifier->ContainsSufficientSamples(&sampleSet)) {
                    if (recognizerMode == ADABOOST_FILTER) {
                        wcscat(errorMessage, L"To build an Adaboost recognizer you need at least 3 positive and 3 negative examples.");
                    } else if (recognizerMode == MOTION_FILTER) {
                        wcscat(errorMessage, L"To build a Motion recognizer you need to to create one or more 'Motion Examples' by making selections while in the current recognizer mode.");
                    } else if (recognizerMode == GESTURE_FILTER) {
                        wcscat(errorMessage, L"To build a gesture recognizer you need to to select a range of frames using the 'Mark In' and 'Mark Out' buttons.");
                    }
		            MessageBox(errorMessage, L"Error Training Recognizer", MB_OK | MB_ICONERROR);
                    break;
                }
                EnableControls(FALSE);
		        classifier->StartTraining(&sampleSet);
                EnableControls(TRUE);
            }
			needToRerunClassifier = true;
            break;
        case IDC_FRAMELEFT:
        case IDC_FRAMERIGHT:
            sliderPosition =
                (long) ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_GETPOS, 0, 0);
            sliderPosition = (wParam==IDC_FRAMELEFT) ? sliderPosition-1 : sliderPosition+1;
            ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_SETPOS, TRUE, sliderPosition);
            OnTrack(0,0,0,bHandled);
			scrubbingVideo = false;
			needToRerunClassifier = true;
            break;
        case IDC_MARKIN:
        case IDC_MARKOUT:
            if (recognizerMode != GESTURE_FILTER) break;
            sliderPosition =
                (long) ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_GETPOS, 0, 0);
            sliderRange = 
                (long) ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_GETRANGEMAX, 0, 0);
            selStart = ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_GETSELSTART, 0, 0);
            selEnd = ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_GETSELEND, 0, 0);
            if (wParam==IDC_MARKIN) {
                selStart = sliderPosition;
                selEnd = (selEnd>sliderPosition) ? selEnd : sliderRange;
            } else {
                selStart = (selStart<sliderPosition) ? selStart : 0;
                selEnd = sliderPosition;
            }
            ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_SETSEL, TRUE, MAKELONG (selStart, selEnd));
            break;
        case IDC_GRABRANGE:
            {
                selStart = ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_GETSELSTART, 0, 0);
                selEnd = ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_GETSELEND, 0, 0);
                // TODO: display informative error message if not enough frames are selected
                if (selEnd - selStart < GESTURE_MIN_TRAJECTORY_LENGTH) break;
                MotionTrack mt = m_videoLoader.GetTrajectoryInRange(selStart, selEnd);
                TrainingSample *sample = new TrainingSample(m_videoLoader.copyFrame, mt, m_sampleListView, m_hImageList, GROUPID_RANGESAMPLES);
                sampleSet.AddSample(sample);
            }
            break;
        case IDC_SHOWBUTTON:
            showGuesses = !showGuesses;
			needToRerunClassifier = true;
            break;
		case IDC_QUICKTEST:
			m_classifierTester.TestClassifierOnVideo(classifier, &m_videoLoader, recognizerMode);
			break;
        case IDC_SAVEFILTER:
            {
                classifier->Save();
                list<Classifier*>::iterator c_iter = find(savedClassifiers.begin(), savedClassifiers.end(), classifier);
                if (c_iter == savedClassifiers.end()) {
                    // current classifier is not in saved list, so we will add it
                    savedClassifiers.push_back(classifier);
                    // also add to the listbox of saved classifiers
                    m_filterSelect.AddSavedFilter(classifier);
                }
                // disable the "train" button until we start a new classifier
                m_filterSelect.GetDlgItem(IDC_TRAINBUTTON).EnableWindow(FALSE);
//                m_filterSelect.GetDlgItem(IDC_SAVEFILTER).EnableWindow(FALSE);
//                m_filterSelect.GetDlgItem(IDC_FILTER_THRESHOLD).EnableWindow(FALSE);
            }
            break;
		case IDC_FILTER_COMBO:
			if (HIWORD(wParam) == CBN_SELCHANGE) { // the user selected a new filter type
				int selectedIndex = ComboBox_GetCurSel(m_filterSelect.GetDlgItem(IDC_FILTER_COMBO));
				recognizerMode = selectedIndex;
				switch(selectedIndex) {
					case COLOR_FILTER:
						ReplaceClassifier(new ColorClassifier());
						break;
					case SHAPE_FILTER:
			            ReplaceClassifier(new ShapeClassifier());
						break;
					case BRIGHTNESS_FILTER:
						ReplaceClassifier(new BrightnessClassifier());
						break;
					case SIFT_FILTER:
						ReplaceClassifier(new SiftClassifier());
						break;
					case ADABOOST_FILTER:
						ReplaceClassifier(new HaarClassifier());
						break;
					case MOTION_FILTER:
						ReplaceClassifier(new MotionClassifier());
						break;
					case GESTURE_FILTER:
						ReplaceClassifier(new GestureClassifier());
						break;
				}
			}
			needToRerunClassifier = true;
			break;
    }
    if (showGuesses && needToRerunClassifier) {
		RunClassifierOnCurrentFrame();
    }
    InvalidateRgn(activeRgn, FALSE);
    return 0;
}

LRESULT CVideoMarkup::OnCustomDraw(int idCtrl, LPNMHDR pnmh, BOOL&) {
    LPNMLVCUSTOMDRAW  lplvcd = (LPNMLVCUSTOMDRAW)pnmh;
    if (lplvcd->nmcd.dwDrawStage == CDDS_PREPAINT) {
        if (draggingIcon) {
            HDC hdc = lplvcd->nmcd.hdc;
            Graphics gListView(hdc);
            if (dragHover) {
                gListView.FillRectangle(&hoverBrush, hoverRect);
            }
        }
    }
    return CDRF_DODEFAULT;
}

LRESULT CVideoMarkup::OnBeginDrag(int idCtrl, LPNMHDR pnmh, BOOL&) {
    POINT p;
    HIMAGELIST hImageListSingle, hImageListMerged;

    int numSelected = ListView_GetSelectedCount(m_sampleListView);
    int iSelection = -1;
    for (int iIndex=0; iIndex<numSelected; iIndex++) {
        iSelection = ListView_GetNextItem(m_sampleListView, iSelection, LVNI_SELECTED);
        if (iIndex == 0) { // first selected icon
            hDragImageList = ListView_CreateDragImage(m_sampleListView, iSelection, &p);
        } else { // subsequent icons
            hImageListSingle = ListView_CreateDragImage(m_sampleListView, iSelection, &p);
            hImageListMerged = ImageList_Merge(hDragImageList, 0, hImageListSingle, 0, iIndex*3, iIndex*3);
            ImageList_Destroy(hDragImageList);
            ImageList_Destroy(hImageListSingle);
            hDragImageList = hImageListMerged;
        }
    }

    ImageList_BeginDrag(hDragImageList, 0, LISTVIEW_SAMPLE_X/2, LISTVIEW_SAMPLE_Y/2);
    POINT pt = ((NM_LISTVIEW*)pnmh)->ptAction;
    RECT listViewRect;
    m_sampleListView.GetClientRect(&listViewRect);
    m_sampleListView.ClientToScreen(&pt);
    m_sampleListView.ClientToScreen(&listViewRect);

    ImageList_DragEnter(GetDesktopWindow(), pt.x, pt.y);
    draggingIcon = TRUE;
    SetCapture();
    return 0;
}

void CVideoMarkup::OpenVideoFile() {
    HCURSOR hOld = SetCursor(LoadCursor(0, IDC_WAIT));
    m_videoLoader.OpenVideoFile(m_hWnd);
	if (m_videoLoader.videoLoaded) {
		EnableControls(TRUE);
        ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_SETRANGEMIN, FALSE, 0);
        ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_SETRANGEMAX, FALSE, m_videoLoader.nFrames-1);
        ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_SETPOS, TRUE, 0);
		m_videoLoader.LoadFrame(0);

        // reset the mask of guesses
		cvSet(m_videoLoader.guessMask, cvScalar(0xFF));

		InvalidateRgn(activeRgn,FALSE);

	    if (recognizerMode == GESTURE_FILTER) {
			// we're in "gesture" mode so we'll need to precompute the trajectories
			m_videoLoader.LearnTrajectories();
		}
	}
    SetCursor(hOld);
}

void CVideoMarkup::OpenSampleFile(char *filename) {
    TrainingSample *sample = new TrainingSample(filename, m_sampleListView, m_hImageList, GROUPID_POSSAMPLES);
    sampleSet.AddSample(sample);
    EnableControls(TRUE);
}

void CVideoMarkup::RecordVideoFile() {
    if (m_videoRecorder.RecordVideoFile(m_hWnd)) {
        // Video was successfully recorded, so we will load it now
        HCURSOR hOld = SetCursor(LoadCursor(0, IDC_WAIT));
        m_videoLoader.OpenVideoFile(m_hWnd, m_videoRecorder.szFileName);
	    if (m_videoLoader.videoLoaded) {
		    EnableControls(TRUE);
		    ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_SETRANGEMIN, FALSE, 0);
		    ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_SETRANGEMAX, FALSE, m_videoLoader.nFrames-1);
		    ::SendDlgItemMessage(m_videoControl, IDC_VIDEOSLIDER, TBM_SETPOS, TRUE, 0);
		    m_videoLoader.LoadFrame(0);
		    InvalidateRgn(activeRgn,FALSE);

		    if (recognizerMode == GESTURE_FILTER) {
				// we're in "gesture" mode so we'll need to precompute the trajectories
				m_videoLoader.LearnTrajectories();
			}
	    }
        SetCursor(hOld);
    }
}

void CVideoMarkup::LoadClassifier(LPWSTR pathname) {

    Classifier *newclassifier = NULL;

    if (wcsstr(pathname, FILE_BRIGHTNESS_SUFFIX) != NULL) {
        newclassifier = new BrightnessClassifier(pathname);
    } else if (wcsstr(pathname, FILE_COLOR_SUFFIX) != NULL) { 
        newclassifier = new ColorClassifier(pathname);
    } else if (wcsstr(pathname, FILE_GESTURE_SUFFIX) != NULL) { 
        newclassifier = new GestureClassifier(pathname);
    } else if (wcsstr(pathname, FILE_HAAR_SUFFIX) != NULL) { 
        newclassifier = new HaarClassifier(pathname);
    } else if (wcsstr(pathname, FILE_MOTION_SUFFIX) != NULL) { 
        newclassifier = new MotionClassifier(pathname);
    } else if (wcsstr(pathname, FILE_SHAPE_SUFFIX) != NULL) { 
        newclassifier = new ShapeClassifier(pathname);
    } else if (wcsstr(pathname, FILE_SIFT_SUFFIX) != NULL) { 
        newclassifier = new SiftClassifier(pathname);
    }

    if (newclassifier != NULL) {
        savedClassifiers.push_back(newclassifier);
        m_filterSelect.AddSavedFilter(newclassifier);
    }
}

void CVideoMarkup::ReplaceClassifier(Classifier *newClassifier) {
    if (!classifier->isOnDisk) {
        // TODO: prompt for confirmation? unsaved classifier will be erased
        // we should also prompt if there are unsaved changes
        delete classifier;
    } else {
        // should already be in the list of saved classifiers (and in the "saved classifiers" list box)
        list<Classifier*>::iterator c_iter = find(savedClassifiers.begin(), savedClassifiers.end(), classifier);
        assert(c_iter != savedClassifiers.end()); 
    }
    classifier = newClassifier;

	// update the filter controls
	m_filterSelect.CheckDlgButton(IDC_SHOWBUTTON, FALSE);
    m_filterSelect.GetDlgItem(IDC_SHOWBUTTON).EnableWindow(classifier->isTrained);
    m_filterSelect.GetDlgItem(IDC_QUICKTEST).EnableWindow(classifier->isTrained);
    m_filterSelect.GetDlgItem(IDC_TRAINBUTTON).EnableWindow(!classifier->isOnDisk);
	m_filterSelect.SelectFilter(classifier->classifierType);
	m_filterSelect.SetThreshold(classifier->threshold);

	// we'll enable the threshold slider and save button to allow tweaking threshold and resaving
    m_filterSelect.GetDlgItem(IDC_SAVEFILTER).EnableWindow(classifier->isTrained);
    m_filterSelect.GetDlgItem(IDC_FILTER_THRESHOLD).EnableWindow(TRUE);


	if (m_videoLoader.videoLoaded) {
		// reset the mask of guesses
		cvSet(m_videoLoader.guessMask, cvScalar(0xFF));

		showGuesses = false;

		// change slider attributes to select either a range or just a single frame, depending on classifier type
		if (recognizerMode == GESTURE_FILTER) {
			m_videoControl.EnableSelectionRange(true);
			m_videoLoader.LearnTrajectories();
		} else {
			m_videoControl.EnableSelectionRange(false);
		}
	}
}

void CVideoMarkup::EmptyTrash() {

    // delete all the samples in the "trash" group
    int iItem = ListView_GetNextItem(m_sampleListView, -1, LVNI_ALL);
    while (iItem != -1) {
        LVITEM lvi;
        lvi.mask = LVIF_IMAGE | LVIF_STATE | LVIF_GROUPID;
        lvi.state = 0;
        lvi.stateMask = 0;
        lvi.iItem = iItem;
        lvi.iSubItem = 0;
        ListView_GetItem(m_sampleListView, &lvi);

        int iNextItem = ListView_GetNextItem(m_sampleListView, iItem, LVNI_ALL);
        if (lvi.iGroupId == GROUPID_TRASH) {

            // remove this sample from the listview
            UINT sampleIdToDelete = ListView_MapIndexToID(m_sampleListView, iItem);
            ListView_DeleteItem(m_sampleListView, iItem);

            // update sample in training set too
            sampleSet.RemoveSample(sampleIdToDelete);

            // indices have changed so we need to start at the beginning of the list again
            iNextItem = ListView_GetNextItem(m_sampleListView, -1, LVNI_ALL);
        }
        iItem = iNextItem;
    }
}

// The wParam stores the new threshold value (from 0 to 100) and the lParam 
// indicates whether or not the slider is currently being dragged
LRESULT CVideoMarkup::OnSetThreshold(UINT, WPARAM wParam, LPARAM lParam, BOOL& ) {
	if ((wParam < 0) || (wParam > 100)) return 0;
	float newThresh = ((float)wParam) / 100.0;
	classifier->threshold = newThresh;
	if (showGuesses && !lParam) {
		RunClassifierOnCurrentFrame();
		InvalidateRgn(activeRgn, FALSE);
	}
	return 0;
}

void CVideoMarkup::RunClassifierOnCurrentFrame() {
	HCURSOR hOld = SetCursor(LoadCursor(0, IDC_WAIT));

	ClassifierOutputData outdata;

    if (recognizerMode == MOTION_FILTER) {
        outdata = ((MotionClassifier*)classifier)->ClassifyMotion(m_videoLoader.GetMotionHistory(), MOTION_NUM_HISTORY_FRAMES);
    } else if (recognizerMode == GESTURE_FILTER) {
		MotionTrack mt = m_videoLoader.GetTrajectoryAtCurrentFrame();
		outdata = ((GestureClassifier*)classifier)->ClassifyTrack(mt);
    } else {
        outdata = classifier->ClassifyFrame(m_videoLoader.copyFrame);
    }
	
	if (outdata.HasVariable("Mask")) {
		IplImage *mask = outdata.GetImageData("Mask");
		cvResize(mask, m_videoLoader.guessMask);
	} else {
		cvZero(m_videoLoader.guessMask);
	}

    SetCursor(hOld);
}

bool CVideoMarkup::GroupTransitionIsAllowed(int origId, int newId) {
	switch (newId) {
		case GROUPID_TRASH:	// always OK to trash something
			return true;
			break;
		case GROUPID_POSSAMPLES:
		case GROUPID_NEGSAMPLES:
			if (origId == GROUPID_POSSAMPLES) return true;
			if (origId == GROUPID_NEGSAMPLES) return true;
			break;
		case GROUPID_MOTIONSAMPLES:
			if (origId == GROUPID_MOTIONSAMPLES) return true;
			break;
		case GROUPID_RANGESAMPLES:
			if (origId == GROUPID_RANGESAMPLES) return true;
			break;
	}
	return false;
}

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
#include "VideoControl.h"

CVideoControl::CVideoControl(CWindow *caller) {
    parent = caller;
    hiMarkin = LoadImage(_AtlBaseModule.GetResourceInstance(), MAKEINTRESOURCE(IDI_MARKIN), IMAGE_ICON, 0, 0, LR_DEFAULTCOLOR);
    hiMarkout = LoadImage(_AtlBaseModule.GetResourceInstance(), MAKEINTRESOURCE(IDI_MARKOUT), IMAGE_ICON, 0, 0, LR_DEFAULTCOLOR);
    hiFrameleft = LoadImage(_AtlBaseModule.GetResourceInstance(), MAKEINTRESOURCE(IDI_FRAMELEFT), IMAGE_ICON, 0, 0, LR_DEFAULTCOLOR);
    hiFrameright = LoadImage(_AtlBaseModule.GetResourceInstance(), MAKEINTRESOURCE(IDI_FRAMERIGHT), IMAGE_ICON, 0, 0, LR_DEFAULTCOLOR);
}

CVideoControl::~CVideoControl(void) {
    DeleteObject(hiMarkin);
    DeleteObject(hiMarkout);
    DeleteObject(hiFrameleft);
    DeleteObject(hiFrameright);
}

LRESULT CVideoControl::OnInitDialog(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled) {
    SendDlgItemMessage(IDC_MARKIN, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hiMarkin);
    SendDlgItemMessage(IDC_MARKOUT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hiMarkout);
    SendDlgItemMessage(IDC_FRAMELEFT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hiFrameleft);
    SendDlgItemMessage(IDC_FRAMERIGHT, BM_SETIMAGE, IMAGE_ICON, (LPARAM)hiFrameright);
    return 0;
}

LRESULT CVideoControl::OnCommand(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled) {
    parent->SendMessage(uMsg, wParam, lParam);
    return 0;
}

LRESULT CVideoControl::OnTrack(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled) {
    parent->SendMessage(uMsg, wParam, lParam);
    return 0;
}

LRESULT CVideoControl::OnEnable(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled) {
    GetDlgItem(IDC_VIDEOSLIDER).EnableWindow(wParam);
    GetDlgItem(IDC_FRAMELEFT).EnableWindow(wParam);
    GetDlgItem(IDC_FRAMERIGHT).EnableWindow(wParam);
    return 1;
}

void CVideoControl::EnableSelectionRange(BOOL enabled) {
    GetDlgItem(IDC_MARKIN).EnableWindow(enabled);
    GetDlgItem(IDC_MARKOUT).EnableWindow(enabled);
    GetDlgItem(IDC_GRABRANGE).EnableWindow(enabled);

    if (enabled) {
        GetDlgItem(IDC_VIDEOSLIDER).SetWindowLong(GWL_STYLE,  WS_CHILD | WS_VISIBLE | TBS_NOTICKS | TBS_ENABLESELRANGE | TBS_BOTH );
        SendDlgItemMessage(IDC_VIDEOSLIDER, TBM_CLEARSEL, TRUE, 0);
    } else {
        GetDlgItem(IDC_VIDEOSLIDER).SetWindowLong(GWL_STYLE,  WS_CHILD | WS_VISIBLE | TBS_NOTICKS | TBS_BOTH );
        SendDlgItemMessage(IDC_VIDEOSLIDER, TBM_CLEARSEL, TRUE, 0);
    }
}


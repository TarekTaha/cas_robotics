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

class CVideoControl : public CDialogImpl<CVideoControl> {
public:
   enum { IDD = IDD_VIDEOCONTROL_DIALOG };

    BEGIN_MSG_MAP(CVideoControl)
        MESSAGE_HANDLER(WM_INITDIALOG, OnInitDialog)
        MESSAGE_HANDLER(WM_COMMAND, OnCommand)
        MESSAGE_HANDLER(WM_HSCROLL, OnTrack)
        MESSAGE_HANDLER(WM_ENABLE, OnEnable)
    END_MSG_MAP()

    CVideoControl(CWindow *parent);
    ~CVideoControl();
    LRESULT OnInitDialog(UINT, WPARAM, LPARAM, BOOL&);
    LRESULT OnTrack(UINT, WPARAM, LPARAM, BOOL&);
    LRESULT OnCommand(UINT, WPARAM, LPARAM, BOOL&);
    LRESULT OnEnable(UINT, WPARAM, LPARAM, BOOL&);
    void EnableSelectionRange(BOOL enabled);

private:
    CWindow *parent;

    HANDLE hiMarkin, hiMarkout, hiFrameleft, hiFrameright;
};

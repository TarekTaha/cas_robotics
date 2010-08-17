/***************************************************************************
 *   Copyright (C) 2006 - 2007 by                                          *
 *      Tarek Taha, CAS-UTS  <tataha@tarektaha.com>                        *
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
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/
#include "statusbar.h"
#include <QStatusBar>
#include <QPushButton>

StatusLogger::StatusLogger(QStatusBar *in_statusBar):
statusBar(in_statusBar)
{
    log.setReadOnly(true);
}

StatusLogger::StatusLogger():
statusBar(NULL)
{
    log.setReadOnly(true);
}

void StatusLogger::showLog()
{
    qDebug("Displaying Log Window");
    log.show();
}
StatusLogger::~StatusLogger()
{
}


void StatusLogger::addStatusMsg(int messageId, int messageType, QString message)
{
    switch (messageType)
    {
    case INFO:
        log.setTextColor(Qt::black);
        break;
    case ERROR:
        log.setTextColor(Qt::red);
        break;
    case WARNING:
        log.setTextColor(Qt::yellow);
        break;
    default:
        log.setTextColor(Qt::black);
    }
    log.append(message);
    if(statusBar)
        statusBar->showMessage(message);
}

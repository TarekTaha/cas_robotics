/***************************************************************************
 *   Copyright (C) 2006 by Waleed Kadous   *
 *   waleed@width   *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include "statusbar.h"

StatusLogger::StatusLogger(QStatusBar *in_statusBar)
{
    statusBar = in_statusBar;
    logButton = new QPushButton("Log");
    connect(logButton, SIGNAL(clicked()),this, SLOT(showLog())); 
    statusBar->addPermanentWidget(logButton);
    log.setReadOnly(true); 
}

void StatusLogger::showLog()
{
    //Doesn't do anything at the mo. 
    qDebug("This should pop up da window."); 
    log.show(); 
}
StatusLogger::~StatusLogger()
{
}


void StatusLogger::addStatusMsg(int messageId, int messageType, QString message)
{
    if(messageType ==0)
    {
		log.setTextColor(Qt::black); 
    }
    if(messageType ==1)
    {
		log.setTextColor(Qt::green); 
    }
    if(messageType ==2)
    {
		log.setTextColor(Qt::black); 
    }
    log.append(message); 
    statusBar->showMessage(message); 
}

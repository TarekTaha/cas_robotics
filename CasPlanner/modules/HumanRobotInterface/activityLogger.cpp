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
#include "activityLogger.h"

ActivityLogger::ActivityLogger()
{
	logFile.open ("tasks.list", ios::out | ios::app | ios::binary);
	if (logFile.is_open())
	{
		printf("\nLogger File Opened");
	}
	else
		printf("\nActivity Logger: File can not be Opened");
}

ActivityLogger::~ActivityLogger()
{
	logFile.close();
}

void ActivityLogger::addState(int state, int observation)
{
	switch(observation)
	{
		case 0:
			logFile << "%d"<<state<<" Up ";
			break;
		case 1:
			logFile << "%d"<<state<<" Down ";
			break;
		case 2:
			logFile << "%d"<<state<<" Right ";
			break;
		case 3:
			logFile << "%d"<<state<<" Left ";
			break;
		case 4:
			logFile << "%d"<<state<<" Nothing ";
			break;
		default:
			perror("ActivityLogger: Unknown Observation!!!");
	}
}

void ActivityLogger::endCurrentTask()
{
	logFile << "\n";
}

void ActivityLogger::startNewTask()
{
	logFile << "\n";
}

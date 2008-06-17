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

ActivityLogger::ActivityLogger():
logFile(NULL)
{
//	QDate date;
//	QTime time;
//	date = date.currentDate();
//	time = time.currentTime();
//	
//	QString suffex = QString("-%1%2%3%4%5.txt").arg(time.minute()).arg(time.hour()).arg(date.day()).arg(date.month()).arg(date.year());
//	logFile = fopen(qPrintable(QString("logs/tasks_list%1").arg(suffex)),"wb+");
	logFile = fopen("logs/tasks_list.txt","a+");
	fprintf(logFile,"\n# Values: State, Observation, State, Observation ....\n");	
	if (logFile)
	{
		printf("\nLogger File Opened");
	}
	else
		printf("\nActivity Logger: File can not be Opened");
}

ActivityLogger::~ActivityLogger()
{
	fclose(logFile);
}

void ActivityLogger::addState(int state, int observation)
{
	fprintf(logFile,"%d",state);
	switch(observation)
	{
		case 0:
			fprintf(logFile," UP ");
			break;
		case 1:
			fprintf(logFile," Down ");
			printf("\n YES");
			break;
		case 2:
			fprintf(logFile," Right ");
			break;
		case 3:
			fprintf(logFile," Left ");
			break;
		case 4:
			fprintf(logFile," Nothing ");
			break;
		default:
			perror("ActivityLogger: Unknown Observation!!!");
	}
}

void ActivityLogger::endCurrentTask()
{
	fprintf(logFile,"\n");
}

void ActivityLogger::startNewTask()
{
	fprintf(logFile,"\n");
}

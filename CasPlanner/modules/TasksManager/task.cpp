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
#include "task.h"

Task::Task()
{
}

Task::Task(QPointF start,QPointF end,QString name)
{
	this->startLocation = start;
	this->endLocation = end;
	this->taskName = name;
        mapPresentation = METRIC;
}

Task::Task(unsigned _startVertex,unsigned _endVertex,unsigned _timeOfDay,QString _name):
        taskName(_name),
        startVertex(_startVertex),
        endVertex(_endVertex),
        timeOfDay(_timeOfDay),
        mapPresentation(TOPOLOGY)
{

}

Task::Task(QPointF start,QPointF end)
{
	this->startLocation = start;
	this->endLocation = end;
        mapPresentation = METRIC;
}

QPointF Task::getStart()
{
    if(mapPresentation == METRIC)
	return this->startLocation;
    else
        return QPointF(0,0);
}

QPointF Task::getEnd()
{
    if(mapPresentation == METRIC)
	return this->endLocation;
    else
        return QPointF(0,0);
}

QString Task::getName()
{
	return this->taskName;
}

unsigned Task::getStartVertex()
{
    return startVertex;
}

unsigned Task::getEndVertex()
{
    return endVertex;
}

unsigned Task::getTimeOfDay()
{
    return timeOfDay;
}

Task::~Task()
{
}



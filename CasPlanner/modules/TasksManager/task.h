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
#ifndef TASK_H
#define TASK_H
#include <QPointF>
#include <QString>

class Task
{
public:
    Task();
    Task(QPointF,QPointF);
    Task(QPointF,QPointF,QString);
    Task(unsigned startVertex,unsigned endVertex,unsigned timeOfDay,QString name);
    ~Task();
    QPointF getStart();
    QPointF getEnd();
    QString getName();
    unsigned getStartVertex();
    unsigned getEndVertex();
    unsigned getTimeOfDay();
    enum TaskMapPresentation{
        METRIC=1,
        TOPOLOGY
    };

    enum TimeOfDay{
        EARLY_MORNING=1,
        MORNING,
        AFTERNOON,
        NIGHT
    };

private:
    QString taskName;
    QPointF startLocation, endLocation;
    unsigned int startVertex,endVertex;
    unsigned int timeOfDay;
    unsigned int mapPresentation;
};

#endif

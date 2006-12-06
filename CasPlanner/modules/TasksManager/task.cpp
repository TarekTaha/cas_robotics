//
// C++ Implementation: task
//
// Description: 
//
//
// Author:  <BlackCoder@localhost.localdomain>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#include "task.h"

Task::Task()
{
}
Task::Task(QPointF start,QPointF end,QString name)
{
	this->startLocation = start;
	this->endLocation = end;
	this->taskName = name;
}

Task::Task(QPointF start,QPointF end)
{
	this->startLocation = start;
	this->endLocation = end;
}

QPointF Task::getStart()
{
	return this->startLocation;
}

QPointF Task::getEnd()
{
	return this->endLocation;
}

QString Task::getName()
{
	return this->taskName;
}

Task::~Task()
{
}



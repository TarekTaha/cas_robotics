//
// C++ Interface: task
//
// Description: 
//
//
// Author:  <BlackCoder@localhost.localdomain>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//
#ifndef TASK_H
#define TASK_H
#include <QPointF>
#include <QString>

/**
	@author  <BlackCoder@localhost.localdomain>
*/
class Task
{
	public:
	    Task();
	    Task(QPointF,QPointF);
	    Task(QPointF,QPointF,QString);	    
	    ~Task();
	    QPointF getStart();
	    QPointF getEnd();
	    QString getName();
	private:
		QString taskName;
		QPointF startLocation, endLocation;
};

#endif

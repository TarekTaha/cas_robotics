#ifndef MAPPAINTER_H_
#define MAPPAINTER_H_

#include <QWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QColor>
#include "utils.h"
#include "PathPlanner.h"
#include "SearchSpaceNode.h"
using namespace CasPlanner;

class MapPainter : public QWidget
{
public:
	MapPainter(QWidget *parent=0);
	virtual ~MapPainter();
	void   setPathEnabled(int);
	Pose   getStart();
	Pose   getEnd();
	Pose   pose;
	QImage getImage();
public slots:
	void drawPath(PathPlanner *,Pose);
	void drawPath(PathPlanner *);
protected:
	QImage image;
	int step;
	QPoint mouse_pos;
	PathPlanner * local_planner,* global_planner;
	bool start_initialized,end_initialized,drawPathEnabled,drawTreeEnabled;
	Pose start,end;
	void paintEvent(QPaintEvent *);
	void mousePressEvent ( QMouseEvent * event );
	void mouseMoveEvent ( QMouseEvent * event );
};

#endif /*MAPPAINTER_H_*/

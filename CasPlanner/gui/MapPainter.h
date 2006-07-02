#ifndef MAPPAINTER_H_
#define MAPPAINTER_H_

#include <QWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QColor>
#include "utils.h"
class MapPainter : public QWidget
{
public:
	MapPainter(QWidget *parent=0);
	virtual ~MapPainter();
	Pose   getStart();
	Pose   getEnd();
	QImage getImage();
protected:
	QImage image;
	int step;
	QPoint mouse_pos;
	bool start_initialized,end_initialized;
	Pose start,end;
	void paintEvent(QPaintEvent *);
	void mousePressEvent ( QMouseEvent * event );
	void mouseMoveEvent ( QMouseEvent * event );
};

#endif /*MAPPAINTER_H_*/

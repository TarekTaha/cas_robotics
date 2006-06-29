#ifndef MAPPAINTER_H_
#define MAPPAINTER_H_

#include <QWidget>
#include <QMouseEvent>
#include <QPainter>
#include <QColor>

class MapPainter : public QWidget
{
public:
	MapPainter(QWidget *parent=0) ;
	virtual ~MapPainter();
protected:
	QImage image;
	QRgb color;
	void paintEvent(QPaintEvent *);
	void mousePressEvent ( QMouseEvent * event );
	void mouseMoveEvent ( QMouseEvent * event );
};

#endif /*MAPPAINTER_H_*/

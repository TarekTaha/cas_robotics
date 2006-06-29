#ifndef MAPPAINTER_H_
#define MAPPAINTER_H_
#include <QWidget>
#include <QMouseEvent>
#include <QPainter>
class MapPainter : public QWidget
{
public:
	MapPainter(QWidget *parent=0) ;
	virtual ~MapPainter();
protected:
	void paintEvent(QPaintEvent *);
	void mousePressEvent ( QMouseEvent * event );
};

#endif /*MAPPAINTER_H_*/

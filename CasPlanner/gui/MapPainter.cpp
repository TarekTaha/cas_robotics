#include "MapPainter.h"

MapPainter::MapPainter(QWidget *parent): QWidget(parent)
{
}

void MapPainter::paintEvent(QPaintEvent *ev)
{
	QImage image;
	if (image.load("/home/BlackCoder/workspace/CasPlanner/resources/casareaicp.png", 0))  
	{                               
		QPainter paint(this);
		paint.drawImage(0, 0, image, 0, 0,image.width(), image.height());
   	}
}

void MapPainter::mousePressEvent ( QMouseEvent * me )
{
	double x = me->x();
	double y = me->y();
    qDebug("Mouse pressed x: %f y: %f",x,y); 
}

MapPainter::~MapPainter()
{
}
